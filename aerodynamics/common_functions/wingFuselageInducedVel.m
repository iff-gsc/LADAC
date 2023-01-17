function v_ind = wingFuselageInducedVel( wing_vortex_pos, Gamma, wing_chord, fuselage_pos )
% wingFuselageInducedVel returns the induced velocity by a
% wing on a fuselage at different centerline positions.
%   The method is based on [1].
% 
% Inputs:
%   wing_vortex_pos     concentrated vectors of horseshoe vortex edge
%                       positions of the wing (3x(N+1) array, where N is
%                       the number of wing panels), in m
%   Gamma               cirulation of each horseshoe vortex (1xN array), in
%                       m^2/s
%   wing_chord          wing chord at the fuselage (scalar), in m
%   fuselage_pos        concentrated vectors of fuselage positions along
%                       the fuselage centerline from front to back (3xM
%                       array, where M is the number of points), in m
% 
% Outputs:
%   v_ind               induced velocity at the fuselage points, where
%                       positive is upwards (1xM array), in m/2
% 
% Literature:
%   [1] Schlichting, H., & Truckenbrodt, E. (2001). Aerodynamik des
%       Flugzeuges. Zweiter Band: Aerodynamik des Tragfl�gels (Teil II),
%       des Rumpfes, der Fl�gel-Rumpf-Anordnung und der Leitwerke. 3.
%       Auflage. Springer-Verlag Berlin Heidelberg.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% implementation according to [1], page 314-315.
wing_n_panel = size( wing_vortex_pos, 2 ) - 1;
A3 = biotSavart( [ones(1,wing_n_panel);zeros(2,wing_n_panel)], ...
    wing_vortex_pos, fuselage_pos );
A = squeeze(A3(3,:,:));

v_ind = -A * Gamma(:);

% v_ind is modified to obtain similar results as in [1], Abb. 10.11 (Kurve
% 3).
% Therefore, the negative step in v_ind is removed.
% Furthermore, the gradient of v_ind is increased in the area -50% ~ 50%
% wing root chord to get the result closer to Abb. 10.11 (Kurve 3) (a 1-cos
% function is used to increase the gradient).
% If you want to see what is going on, you should debug this function and
% plot the first result of v_ind as well as the modified v_ind.
% Note that the results of Abb. 10.11 (Kurve 3) at the rear of the fuselage
% could only be obtained if the width of the rear fuselage was enhanced to
% account for viscous lift. Else, the lift at the rear fuselage will be
% much more negative.
wing_root_idx = round( interp1( wing_vortex_pos(2,:), 1:(wing_n_panel+1), mean( fuselage_pos(2,:) ) ) );
wing_root_x = wing_vortex_pos(1,wing_root_idx);

is_step = diff(v_ind) < 0;
idx_step = find(is_step==1);
idx_overlay = find( fuselage_pos(1,:)<wing_root_x+wing_chord*3/4 & fuselage_pos(1,:)>wing_root_x-wing_chord*2/4 );
idx_step = idx_overlay(1):idx_step(end);
idx_up = idx_step(end)+1:length(v_ind);
idx_stay = 1:idx_overlay(1);
idx_interp = idx_stay(end)+1 : idx_up(1);
diff_interp1 = mean([diff(v_ind(idx_stay(end-1:end))),diff(v_ind(idx_up(1:2)))]);
diff_interp = diff_interp1 .* ( ...
    0.5*( 1 - 0.5*cos( ...
        2*pi * (fuselage_pos(1,idx_interp)-fuselage_pos(1,idx_interp(1)-1)) ...
        /(fuselage_pos(1,idx_interp(end)+1)-fuselage_pos(1,idx_interp(1)-1) ) ...
        ) ) ...
    + linspace( 1, 1, length(idx_interp) ) ...
    );
diff_up = sum(diff_interp) + v_ind(idx_stay(end)) - v_ind(idx_up(1));
v_ind(idx_up) = v_ind(idx_up) + diff_up;
v_ind(idx_interp) = v_ind(idx_stay(end)) + cumsum(diff_interp);

end