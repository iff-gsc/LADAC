function [traj, traj_valid, uvwxb, AnormAlfaRhoPhi, state_vec]  = trajFromWaypointsIterative(traj_last,traj_empty, waypoints,num_wp,cycle,trig_update, uvwxb, AnormAlfaRhoPhi, state_vec,  traj_empty_struct)
% traj_empty_struct is a parameter, known at compile time
%traj_empty = traj_empty_struct;

degree = traj_empty_struct.polynomial_degree;
%degree = traj.polynomial_degree;
%datatype = superiorfloat(state_vec);

traj     = traj_last;

state    = state_vec(1);
b_size   = state_vec(2);
axis_sel = state_vec(3);
num_of_splines = state_vec(4);
traj_valid = state_vec(5);
residuum = state_vec(6);

%%
num_wp = min([num_wp, traj.num_sections_max, size(waypoints,2)]);

%% axis_sel = 0: Initialize
if (trig_update == 1) && (state == 0) && (axis_sel == 0)

    traj_valid = zeros(1, superiorfloat(state_vec));
    traj = traj_empty;
    
    % Select the first axis and set state to zero
    axis_sel = ones(1, superiorfloat(state_vec));
    state    = zeros(1, superiorfloat(state_vec));
    
    
elseif (axis_sel >= 1) && (axis_sel <= 3)
    
    % State zero is initalizing
    if state == 0
        points = waypoints(axis_sel,1:num_wp);
        
        [b, num_of_splines] = polyInterpolationb(points, degree, cycle);
        
        A = @(x, a) polyInterpolationAx(num_of_splines, degree, cycle, x, a);
        [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ladac_lsqr_init(A, b);
        
        state = ones(1, superiorfloat(state_vec));
        b_size = length(b) * ones(1, superiorfloat(b));
        
        % Copy all temporary variables into loop buffer
        uvwxb(1:b_size, 1) = b;
        uvwxb(1:b_size, 2) = u;
        uvwxb(1:b_size, 3) = v;
        uvwxb(1:b_size, 4) = w;
        uvwxb(1:b_size, 5) = x;   
        AnormAlfaRhoPhi = [Anorm; alfa; rhobar; phibar];
        
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
            ladac_lsqr_iterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar);
        
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
        residuum = norm( A(x,1) - b );
        
        % Check if finished
        if( residuum < 1e-3)
            state = single(1000);
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
% state_vec(1) = state * ones(1, superiorfloat(state_vec));
% state_vec(2) = b_size * ones(1, superiorfloat(state_vec));
% state_vec(3) = axis_sel * ones(1, superiorfloat(state_vec));
% state_vec(4) = num_of_splines * ones(1, superiorfloat(state_vec));
% state_vec(5) = traj_valid * ones(1, superiorfloat(state_vec));

state_vec = single([state; b_size; axis_sel; num_of_splines; traj_valid; residuum]);

end


% % Check if the polynomial degree is even, if yes increase it by one to get
% % the same number of constraints on the left and right side.
% if(~mod(degree,2))
%     degree = degree + 1;
% end
%
% % Calculate the coefficients
% [coeffs_x, num_of_splines] = polyInterpolation(waypoints(1,:), ...
%                              degree, cycle, 0, 0);
% [coeffs_y, ~] = polyInterpolation(waypoints(2,:), degree, cycle, 0, 0);
% [coeffs_z, ~] = polyInterpolation(waypoints(3,:), degree, cycle, 0, 0);
%
% % Configure trajectory struct with information about the path
% traj.num_sections_set(:)   = num_of_splines;
% traj.is_repeated_course(:) = cycle;
% traj.polynomial_degree  = degree;
%
% % Copy the coefficients into the spline trajectory
% num_of_coeffs = degree + 1;
%
% % Copty the coefficients into the traj struct
% for i=1:num_of_splines
%     idx_beg = num_of_coeffs*(i-1)+1;
%     idx_end = num_of_coeffs*(i);
%     traj.sections(i).pos_x(:) = coeffs_x(idx_beg:idx_beg+5);
%     traj.sections(i).pos_y(:) = coeffs_y(idx_beg:idx_beg+5);
%     traj.sections(i).pos_z(:) = coeffs_z(idx_beg:idx_beg+5);
% end
%
% % Update arc length for each section
% traj = trajSetArcLength(traj);

