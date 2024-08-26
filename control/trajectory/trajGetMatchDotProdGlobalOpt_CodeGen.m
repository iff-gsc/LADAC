function [section_idx_ret, error, t_ret] = trajGetMatchDotProdGlobalOpt(traj, position, active_section)
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

section_idx_ret = zeros(1,1,superiorfloat(position));
t_ret = zeros(1,1,superiorfloat(position));

for k = first_checked_section:last_checked_section
    
    t = zeros(1,1,superiorfloat(position));
    %    trajSection = traj.sections(k);
    
    section_idx = single(k);
    
    %% Projection Iteration
    
    fun_x = traj.sections(section_idx).pos_x;
    fun_y = traj.sections(section_idx).pos_y;
    fun_z = traj.sections(section_idx).pos_z;
    
    %     %Prefilter candidates:
    %
    %     is_candidate = true;
    %
    %     x = [xf; yf; zf];
    %
    %     [a, da] = evaluate_polynomials(fun_x, fun_y, fun_z, 0, degree);
    %     [b, db] = evaluate_polynomials(fun_x, fun_y, fun_z, 1, degree);
    %
    %     bma = b-a;
    %     bma_dot_a = dot(bma, da);
    %     bma_dot_b = dot(bma, db);
    %
    %     xma_dot_a = dot(x-a, da);
    %     bmx_dot_b = dot(b-x, db);
    
    %     if(bma_dot_a >= 0) && (bma_dot_b >= 0)
    %          %is_candidate = true;
    %          if (xma_dot_a >= 0) && (bmx_dot_b >= 0)
    %          else
    %               is_candidate = false;
    %          end
    %
    %     if (bma_dot_a < 0) && (bma_dot_b > 0)
    %          if (xma_dot_a >= 0) || (bmx_dot_b < 0)
    %          else
    %              is_candidate = false;
    %          end
    %     end
    %
    
    for i=1:20
        
%         [f1, df1, f2, df2, f3, df3] = evaluate_polynomials(fun_x, fun_y, fun_z, t);
        
        % Initialize polynomial and derivative values
        f1 = fun_x(1);  % Initialize p1 with the leading coefficient of the first polynomial
        df1 = zeros(1,1,superiorfloat(f1));         % Initialize dp1 with zero
        
        f2 = fun_y(1);  % Initialize p2 with the leading coefficient of the second polynomial
        df2 = zeros(1,1,superiorfloat(f1));          % Initialize dp2 with zero
        
        f3 = fun_z(1);  % Initialize p3 with the leading coefficient of the third polynomial
        df3 = zeros(1,1,superiorfloat(f1));          % Initialize dp3 with zero
        
        % Loop through the coefficients
        for iter = 2:degree
            % Horner's method for polynomial evaluation and derivative
            df1 = df1 * t + f1; % Derivative update for first polynomial
            f1 = f1 * t + fun_x(iter); % Polynomial update for first polynomial
            
            df2 = df2 * t + f2; % Derivative update for second polynomial
            f2 = f2 * t + fun_y(iter); % Polynomial update for second polynomial
            
            df3 = df3 * t + f3; % Derivative update for third polynomial
            f3 = f3 * t + fun_z(iter); % Polynomial update for third polynomial
        end
        
        f = [f1; f2; f3];
        
        eval_fun = (df1*(f1-xf) + df2*(f2-yf) + df3*(f3-zf));
        
        % Calculate efficiently: norm(f_dt)^2 = dot(f_dt, f_dt)
        norm_d_dt_squared = df1*df1+df2*df2+df3*df3;
        
        % Check for possible division by zero, even though this case cannot
        % occur according to the definition of the path.
        if(norm_d_dt_squared < 1e-12)
             break;
        end
        
        max_step = cast(0.1,'like',f1); 
        %step = -eval_fun / norm_d_dt_squared;
        
        step = -eval_fun / norm_d_dt_squared ;
        
        step = min(max(step, -max_step), max_step);
        t = t + step;
        
        % Break loop
        if(abs(eval_fun) < 1e-12)
            
            distance = norm(f-position);
            if( distance < error)
                error = distance;
                section_idx_ret = section_idx;
                t_ret = min(max(t,0),1);
            end
            break;
        end
        
        % Step to previous/next section
        if(t < 0) || (t > 1.00)
            if(t < 0)
                t = max(t+1, 1.00);
                section_idx = section_idx - 1;
                if(section_idx < 1)
                    if traj.is_repeated_course
                        section_idx = traj.num_sections_set;
                    else
                        section_idx = ones(1,superiorfloat(position));
                        t = zeros(1,superiorfloat(position));
                    end
                end
            else
                t = min(t-1, 0.00);
                section_idx = section_idx + 1;
                if(section_idx > traj.num_sections_set)
                    if traj.is_repeated_course
                        section_idx = ones(1,superiorfloat(position));
                    else
                        section_idx = traj.num_sections_set;
                        t = ones(1,superiorfloat(position));
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

% function [f, df] = evaluate_polynomials(fun_x, fun_y, fun_z, t, degree)
% 
% % Initialize polynomial and derivative values
% f1 = fun_x(1);  % Initialize p1 with the leading coefficient of the first polynomial
% df1 = 0;         % Initialize dp1 with zero
% 
% f2 = fun_y(1);  % Initialize p2 with the leading coefficient of the second polynomial
% df2 = 0;         % Initialize dp2 with zero
% 
% f3 = fun_z(1);  % Initialize p3 with the leading coefficient of the third polynomial
% df3 = 0;         % Initialize dp3 with zero
% 
% % Loop through the coefficients
% for iter = 2:degree
%     % Horner's method for polynomial evaluation and derivative
%     df1 = df1 * t + f1; % Derivative update for first polynomial
%     f1 = f1 * t + fun_x(iter); % Polynomial update for first polynomial
%     
%     df2 = df2 * t + f2; % Derivative update for second polynomial
%     f2 = f2 * t + fun_y(iter); % Polynomial update for second polynomial
%     
%     df3 = df3 * t + f3; % Derivative update for third polynomial
%     f3 = f3 * t + fun_z(iter); % Polynomial update for third polynomial
% end
% 
% f = [f1; f2; f3];
% df = [df1; df2; df3];
% 
% end
% 
% function [f, df, d2f] = evaluate_polynomials_with_second_derivative(coeff1, coeff2, coeff3, t)
%     % coeff1, coeff2, coeff3: Coefficients of the three polynomials (vectors of length 6)
%     % t: The value at which to evaluate the polynomials, first and second derivatives
% 
%     % Initialize polynomial, first derivative, and second derivative values
%     p1 = coeff1(1);  % Initialize p1 with the leading coefficient of the first polynomial
%     dp1 = 0*coeff1(1);         % Initialize dp1 with zero (first derivative of first polynomial)
%     d2p1 = 0*coeff1(1);        % Initialize d2p1 with zero (second derivative of first polynomial)
% 
%     p2 = coeff2(1);  % Initialize p2 with the leading coefficient of the second polynomial
%     dp2 = 0*coeff2(1);         % Initialize dp2 with zero (first derivative of second polynomial)
%     d2p2 = 0*coeff2(1);        % Initialize d2p2 with zero (second derivative of second polynomial)
% 
%     p3 = coeff3(1);  % Initialize p3 with the leading coefficient of the third polynomial
%     dp3 = 0*coeff3(1);         % Initialize dp3 with zero (first derivative of third polynomial)
%     d2p3 = 0*coeff3(1);        % Initialize d2p3 with zero (second derivative of third polynomial)
% 
%     % Loop through the coefficients
%     for i = 2:length(coeff1)
%         % Horner's method for second derivative calculation
%         d2p1 = d2p1 * t + 2 * dp1; % Second derivative update for first polynomial
%         dp1 = dp1 * t + p1;        % First derivative update for first polynomial
%         p1 = p1 * t + coeff1(i);   % Polynomial update for first polynomial
% 
%         d2p2 = d2p2 * t + 2 * dp2; % Second derivative update for second polynomial
%         dp2 = dp2 * t + p2;        % First derivative update for second polynomial
%         p2 = p2 * t + coeff2(i);   % Polynomial update for second polynomial
% 
%         d2p3 = d2p3 * t + 2 * dp3; % Second derivative update for third polynomial
%         dp3 = dp3 * t + p3;        % First derivative update for third polynomial
%         p3 = p3 * t + coeff3(i);   % Polynomial update for third polynomial
%     end
%     
%     f = [p1; p2; p3];
%     df = [dp1; dp2; dp3];
%     d2f = [d2p1; d2p2; d2p3];
% end