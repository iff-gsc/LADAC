function [traj, traj_valid, uvwxb, AnormAlfaRhoPhi, state_vec]  = trajFromWaypointsIterative(traj_last,traj_empty, waypoints,num_wp,cycle,trig_update, uvwxb, AnormAlfaRhoPhi, state_vec, traj_empty_struct)
% trajFromWaypointsIterative implements a state machine for the iterative 
%   computation of the trajectory from given waypoints in Simulink.
%   The function returns the filled trajectory struct representing a flight
%   path given by the waypoints
%
% Inputs:
%   traj_last       trajectory struct from previous time step, see trajInit
%
%   traj_empty      empty trajectory struct that is used for initialization
%
%   waypoints       trajectory waypoints [x; y; z] in local geodetic
%                   coordinate system
%                   (3xN vector), in m
%
%   num_wp          the total number of waypoints
%                   (scalar), dimensionless
%
%   cycle           enable automatic repetition of the given course
%                   to ensure that the derivatives of the first and last
%                   point are the same.
%                   (boolean)
%
%   degree 		    degree of internal polynomial representation this value
%                   should be an odd number to ensure symmetrically
%                   boundary conditions on every knot point (1,3,5,...)
%                   (scalar), dimensionless
%
%   trig_update     signal indicating that the waypoints have been changed
%                   and the trajectory needs to be recalculated
%                   (boolean)
%
%   uvwxb           result of the auxiliary vectors of the last iteration
%                   (Nx5 vector), dimensionless
%
%   AnormAlfaRhoPhi result of the scalar sizes of the last iteration
%                   (4x1 vector), dimensionless
%
%   state_vec       State vector of the last iteration with the variables:
%                   state: Indicates initialization(0) or iteration(1)
%                   b_size: size of the right-hand-side
%                   axis_sel: Shows the current dimension (0=none,1=x,...)
%                   num_of_splines: Numer of spline segments
%                   traj_valid: Indicates the completion of the calculation
%                   residuum: Shows the current residuum of ||(Ax-b)||_2
%                   (6x1 vector), dimensionless
%   
%   traj_empty_struct empty trajectory struct that is used for initialization
%
% Outputs:
%
%   traj            trajectory struct, see trajInit
%
%   traj_valid      Indicates the completion of the calculation
%                   (scalar), dimensionless
%
%   uvwxb           result of the auxiliary vectors of this iteration
%                   (Nx5 vector), dimensionless
%
%   AnormAlfaRhoPhi result of the scalar sizes of this iteration
%                   (4x1 vector), dimensionless
%
%   state_vec       see definition above at the inputs 
%                   (6x1 vector), dimensionless
%
% Syntax:
%   [traj, traj_valid, uvwxb, AnormAlfaRhoPhi, state_vec] = ...
%   trajFromWaypointsIterative(traj_last,traj_empty, waypoints,num_wp, ...
%   cycle,trig_update, uvwxb, AnormAlfaRhoPhi, state_vec, ...
%   traj_empty_struct)
%
% See also: trajInit, traj_from_waypoints_example
%
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% traj_empty_struct is a parameter, known at compile time
% traj_empty = traj_empty_struct;

degree          = traj_empty_struct.polynomial_degree;
traj            = traj_last;

state           = state_vec(1);
b_size          = state_vec(2);
axis_sel        = state_vec(3);
num_of_splines  = state_vec(4);
traj_valid      = state_vec(5);
residuum        = state_vec(6);

datatype = superiorfloat(state_vec);

num_wp = min([num_wp, traj.num_sections_max, size(waypoints,2)]);

%% axis_sel = 0: Initialize
if (trig_update == 1) && (state == 0) && (axis_sel == 0)

    traj_valid = zeros(1, superiorfloat(state_vec));
    traj = traj_empty;
    
    % check waypoints are valid   
    if trajValidateWaypoints(waypoints, num_wp, cycle)
        
        % Select the first axis and set state to zero
        axis_sel = ones(1, superiorfloat(state_vec));
        state    = zeros(1, superiorfloat(state_vec)); 
    end
    
elseif (axis_sel >= 1) && (axis_sel <= 3)
    
    % State zero is initalizing
    if state == 0
        points = waypoints(axis_sel,1:num_wp);
        
        [b, num_of_splines] = polyInterpolationb(points, degree, cycle);
        
        A = @(x, a) polyInterpolationAx(num_of_splines, degree, cycle, x, a);
        [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ladacLsqrInit(A, b);
        
        state = ones(1, superiorfloat(state_vec));
        b_size = length(b) * ones(1, superiorfloat(b));
        
        % Copy all temporary variables into loop buffer
        uvwxb(1:b_size, 1) = b;
        uvwxb(1:b_size, 2) = u;
        uvwxb(1:b_size, 3) = v;
        uvwxb(1:b_size, 4) = w;
        uvwxb(1:b_size, 5) = x;   
        AnormAlfaRhoPhi = [Anorm; alfa; rhobar; phibar];
        
        % Calculate inital error
        residuum = norm( A(x,1) - b );
        
    % States above zero mean iterating
    elseif state < 1000
        
        b = uvwxb(1:b_size, 1);
        u = uvwxb(1:b_size, 2);
        v = uvwxb(1:b_size, 3);
        w = uvwxb(1:b_size, 4);
        x = uvwxb(1:b_size, 5);
        
        Anorm  = AnormAlfaRhoPhi(1);
        alfa   = AnormAlfaRhoPhi(2);
        rhobar = AnormAlfaRhoPhi(3);
        phibar = AnormAlfaRhoPhi(4);
        
        A = @(x, a) polyInterpolationAx(num_of_splines, degree, cycle, x, a);
        
        [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ...
            ladacLsqrIterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar);
        
        % Copy all temporary variables into loop buffer
        uvwxb(1:b_size, 1) = b;
        uvwxb(1:b_size, 2) = u;
        uvwxb(1:b_size, 3) = v;
        uvwxb(1:b_size, 4) = w;
        uvwxb(1:b_size, 5) = x;   
        AnormAlfaRhoPhi = [Anorm; alfa; rhobar; phibar];
        
        % Increase state by one
        state = state + 1;
        
        % Calculate residuum
        residuum_last = residuum;
        residuum = norm( A(x,1) - b );
        
        % Check if finished
        if (residuum < 1e-3) || (residuum_last < residuum)
            if strcmp(datatype, 'single')
                state = single(1000);
            else
                state = double(1000);
            end
        end
        
    % In the ast state copy the values from x into traj struct
    else
        
        x = uvwxb(1:b_size, 5);
        
        % Configure trajectory struct with information about the path
        traj.num_sections_set   = num_of_splines;
        traj.is_repeated_course = logical(cycle);
        
        % Copy the coefficients into the spline trajectory
        num_of_coeffs = degree + 1;
        
        % Copy the coefficients into the traj struct
        for i=1:num_of_splines
            idx_beg = num_of_coeffs*(i-1)+1;
            idx_end = idx_beg + degree;
            
            if axis_sel == 1
                traj.sections(i).pos_x(:) = x(idx_beg:idx_end);
            elseif axis_sel == 2
                traj.sections(i).pos_y(:) = x(idx_beg:idx_end);
            elseif axis_sel == 3
                traj.sections(i).pos_z(:) = x(idx_beg:idx_end);
            end
        end
        
        % Select the next axis and reset state to zero
        state    = zeros(1, superiorfloat(state_vec));
        axis_sel = axis_sel + 1;
        
    end
        
% Calculation is done, calculate arc length for the trajectory
elseif  (axis_sel == 4)
    
    % Update arc length for each section
    traj = trajSetArcLength(traj);
    
    % Trajectory is valid and useable 
    traj_valid = ones(1, superiorfloat(state_vec));
    
    % Reset state-machine
    axis_sel = zeros(1, superiorfloat(state_vec));
    state    = zeros(1, superiorfloat(state_vec));
    
end


%% Finalize

if strcmp(datatype, 'single')
    state_vec = single([state; b_size; axis_sel; num_of_splines; traj_valid; residuum]);
else
    state_vec = double([state; b_size; axis_sel; num_of_splines; traj_valid; residuum]);
end

end


