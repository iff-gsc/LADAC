function [section_idx_ret, error, t_ret] = trajGetMatchNewton(traj, position, active_section)
% trajGetMatch computes the nearest point on the trajectory
%   The function returns the point on the trajectory that has minimum
%   euclidean norm in respect to the given position.
%
% Inputs:
%   traj            trajectory struct, see trajInit
%
%   position        vehicle position [x; y; z] in local geodetic
%                   coordinate system
%                   (3x1 vector), in m
%
%   active_section 	*placeholder for later use*
%                   current section of the trajectory
%                  	(scalar), dimensionless
% Outputs:
%
%   section_idx     index of the trajectory section that contains the point
%                   of minimum eulidean distance
%
%   error           the minimum euclidean distance between the given
%                   position and the trajectory
%                   (scalar), in m
%
%   t               dimensionless time parameter that correnspons to the
%                   trajectory point of minimum eulidean distance
%                	(scalar), [0-1]
%
% Syntax:
%   [section_idx, error, t] = trajGetMatch(traj, position, active_section)
%
% See also: trajGetMatch, trajCreateFromWaypoints
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
%
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
first_checked_section = 1;
last_checked_section = traj.num_sections_set;

traj_section = trajGetSection(traj, 1);

%t = zeros(1,superiorfloat(position));
%section_idx = ones(1,superiorfloat(position));
error = norm(position - trajSectionGetPos(traj_section, 0));

degree = length(traj.sections(1).pos_x);

%t_array = zeros(1,superiorfloat(position));

%trajSection = trajGetSection(traj, section_idx);

xf = position(1);
yf = position(2);
zf = position(3);

section_idx_ret = 0;
t_ret = 0;

for k = first_checked_section:last_checked_section
    
    t = 0;
    %    trajSection = traj.sections(k);
    
    section_idx = k;
    
    %% Projection Iteration
    
    fun_x = traj.sections(section_idx).pos_x;
    fun_y = traj.sections(section_idx).pos_y;
    fun_z = traj.sections(section_idx).pos_z;
    
    for i=1:10
        
        %   [f, f_dt, f_dt2] = evaluate_polynomials_with_second_derivative(fun_x, fun_y, fun_z, t);
        
        %%
        
        % Initialize polynomial, first derivative, and second derivative values
        p1 = fun_x(1);  % Initialize p1 with the leading coefficient of the first polynomial
        dp1 = 0;         % Initialize dp1 with zero (first derivative of first polynomial)
        d2p1 = 0;        % Initialize d2p1 with zero (second derivative of first polynomial)
        
        p2 = fun_y(1);  % Initialize p2 with the leading coefficient of the second polynomial
        dp2 = 0;         % Initialize dp2 with zero (first derivative of second polynomial)
        d2p2 = 0;        % Initialize d2p2 with zero (second derivative of second polynomial)
        
        p3 = fun_z(1);  % Initialize p3 with the leading coefficient of the third polynomial
        dp3 = 0;         % Initialize dp3 with zero (first derivative of third polynomial)
        d2p3 = 0;        % Initialize d2p3 with zero (second derivative of third polynomial)
        
        % Loop through the coefficients
        for i = 2:length(fun_x)
            % Horner's method for second derivative calculation
            d2p1 = d2p1 * t + 2 * dp1; % Second derivative update for first polynomial
            dp1 = dp1 * t + p1;        % First derivative update for first polynomial
            p1 = p1 * t + fun_x(i);   % Polynomial update for first polynomial
            
            d2p2 = d2p2 * t + 2 * dp2; % Second derivative update for second polynomial
            dp2 = dp2 * t + p2;        % First derivative update for second polynomial
            p2 = p2 * t + fun_y(i);   % Polynomial update for second polynomial
            
            d2p3 = d2p3 * t + 2 * dp3; % Second derivative update for third polynomial
            dp3 = dp3 * t + p3;        % First derivative update for third polynomial
            p3 = p3 * t + fun_z(i);   % Polynomial update for third polynomial
        end
        
        f = [p1; p2; p3];
        f_dt = [dp1; dp2; dp3];
        f_dt2 = [d2p1; d2p2; d2p3];
        
        %%
        
        eval_fun = (f_dt(1)*(f(1)-xf) + f_dt(2)*(f(2)-yf) + f_dt(3)*(f(3)-zf));
        
        eval_fun_dt = (f_dt2(1)*(f(1)-xf) + f_dt2(2)*(f(2)-yf) + f_dt2(3)*(f(3)-zf)...
            + f_dt(1)^2 + f_dt(2)^2 + f_dt(3)^2);
        
        % Check for convergence
        
        if abs(eval_fun_dt) < 1e-16
            eval_fun_dt(:) = 1e-8;
        end
        
        step = -divideFinite(eval_fun, abs(eval_fun_dt));
        
        step = min(max(step, -0.2), 0.2);
        t = t + step;
        
        % Break loop
        %if(abs(eval_fun) < 1e-12)
        
        distance = norm(f-position);
        if( distance < error)
            error = distance;
            section_idx_ret = section_idx;
            t_ret = min(max(t,0),1);
        end
        %    break;
        %end
        
        % Step to previous/next section
        if(t < 0) || (t > 1.00)
            if(t < 0)
                t = max(t+1, 1.00);
                section_idx = section_idx - 1;
                if(section_idx < 1)
                    if traj.is_repeated_course
                        section_idx = traj.num_sections_set;
                    else
                        section_idx = 1;
                        t = 0;
                    end
                end
            else
                t = min(t-1, 0.00);
                section_idx = section_idx + 1;
                if(section_idx > traj.num_sections_set)
                    if traj.is_repeated_course
                        section_idx = 1;
                    else
                        section_idx = traj.num_sections_set;
                        t = 1;
                    end
                end
            end
            
            fun_x = traj.sections(section_idx).pos_x;
            fun_y = traj.sections(section_idx).pos_y;
            fun_z = traj.sections(section_idx).pos_z;
            
        end
        
    end
end
end

function [f, df, d2f] = evaluate_polynomials_with_second_derivative(fun_x, fun_y, fun_z, t)
% coeff1, coeff2, coeff3: Coefficients of the three polynomials (vectors of length 6)
% t: The value at which to evaluate the polynomials, first and second derivatives

% Initialize polynomial, first derivative, and second derivative values
p1 = fun_x(1);  % Initialize p1 with the leading coefficient of the first polynomial
dp1 = 0*fun_x(1);         % Initialize dp1 with zero (first derivative of first polynomial)
d2p1 = 0*fun_x(1);        % Initialize d2p1 with zero (second derivative of first polynomial)

p2 = fun_y(1);  % Initialize p2 with the leading coefficient of the second polynomial
dp2 = 0*fun_y(1);         % Initialize dp2 with zero (first derivative of second polynomial)
d2p2 = 0*fun_y(1);        % Initialize d2p2 with zero (second derivative of second polynomial)

p3 = fun_z(1);  % Initialize p3 with the leading coefficient of the third polynomial
dp3 = 0*fun_z(1);         % Initialize dp3 with zero (first derivative of third polynomial)
d2p3 = 0*fun_z(1);        % Initialize d2p3 with zero (second derivative of third polynomial)

% Loop through the coefficients
for i = 2:length(fun_x)
    % Horner's method for second derivative calculation
    d2p1 = d2p1 * t + 2 * dp1; % Second derivative update for first polynomial
    dp1 = dp1 * t + p1;        % First derivative update for first polynomial
    p1 = p1 * t + fun_x(i);   % Polynomial update for first polynomial
    
    d2p2 = d2p2 * t + 2 * dp2; % Second derivative update for second polynomial
    dp2 = dp2 * t + p2;        % First derivative update for second polynomial
    p2 = p2 * t + fun_y(i);   % Polynomial update for second polynomial
    
    d2p3 = d2p3 * t + 2 * dp3; % Second derivative update for third polynomial
    dp3 = dp3 * t + p3;        % First derivative update for third polynomial
    p3 = p3 * t + fun_z(i);   % Polynomial update for third polynomial
end

f = [p1; p2; p3];
df = [dp1; dp2; dp3];
d2f = [d2p1; d2p2; d2p3];
end