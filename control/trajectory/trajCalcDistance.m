function dist = trajCalcDistance(traj_section, R, position, T_vec, t, penalty)

degree = length(traj_section.pos_x);

t_vec = ones(degree, 1);

for i = degree-1:-1:1
    t_vec(i) = t_vec(i+1) * t;
end

% Connectiong line
r_ac_path = [(traj_section.pos_x * t_vec) - position(1);
    (traj_section.pos_y * t_vec) - position(2);
    (traj_section.pos_z * t_vec) - position(3)];

% Calculate distance
dist = norm(r_ac_path);

% Enable penalty function for alignment
if(penalty)
    
    t_vec_diff = zeros(degree, 1);
    
    for i = degree-1:-1:1
        t_vec_diff(i) = t_vec(i+1) * (degree - i);
    end
    
    % Calculate first derivative
    first_deriv = [traj_section.pos_x * t_vec_diff;
                   traj_section.pos_y * t_vec_diff;
                   traj_section.pos_z * t_vec_diff];
    
    % Scale the vectors to unit length
%     r_ac_path_norm   = r_ac_path / max(norm(r_ac_path), 1e-6);
    first_deriv_norm = first_deriv / max(norm(first_deriv), 1e-6);
    T_vec_norm       = T_vec / max(norm(T_vec), 1e-6);
    
    
    % Measure of the error of alignment of the two vectors to each other
    align_cos = 1.0 - norm(cross(T_vec_norm, first_deriv_norm));
    align_sin = dot(T_vec_norm, first_deriv_norm);
     
    
    % Add two times the turn radius for wrong alignment
    if(align_sin < 0)
         dist = dist + max(2*R - dist, 0) * 2*R * align_cos;
    end
    
    % Measure of alignment of aircraft's tangent to connecting line
%     p1 = 0.5 * (1.0 - dot(r_ac_path_norm, T_vec_norm));
%     p2 = 0.5 * (1.0 - dot(r_ac_path_norm, first_deriv_norm));  
    
    % Add two times the turn radius for wrong alignment
%     dist = dist + 2*R * p1*p2;
    
end

end

