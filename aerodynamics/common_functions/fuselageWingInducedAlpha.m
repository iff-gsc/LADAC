function Delta_alpha_rel = fuselageWingInducedAlpha( R, y )
% fuselageWingInducedAlpha return the relative induced angle of attack by a
% fuselage on a wing at different spanwise positions.
%   The method is based on [1]. However, it was found that the presented
%   theory in [1], Fig. 10.17 does not really match the presented measured
%   data in [1], Fig. 10.16.
%   That is why "Kurve 1" is used for the outer fuselage part (but "Kurve
%   1" was multiplied with the factor 1.6!). For the inner fuselage part,
%   the function is also based on "Kurve 1", but modified even stronger:
%   The exponent of 2 was replaced with an exponent of 6. Moreover, the
%   curve goes from -2 to 1.6 instead of from -1 to 1.
% 
% Inputs:
%   R           fuselage radius (scalar), in m
%   y           spanwise wing position (1xN array), in m
% 
% Outputs:
%   Delta_alpha_rel     relative induced angle of attack (Delta
%                       alpha)/(alpha_inf) (1xN array), in m
% 
% Literature:
%   [1] Schlichting, H., & Truckenbrodt, E. (2001). Aerodynamik des
%       Flugzeuges. Zweiter Band: Aerodynamik des Tragfluegels (Teil II),
%       des Rumpfes, der Fluegel-Rumpf-Anordnung und der Leitwerke. 3.
%       Auflage. Springer-Verlag Berlin Heidelberg.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

Delta_alpha_rel = zeros( size(y) );

% indices where y is inside fuselage area
idx = abs(y)<R;

Delta_alpha_rel(idx) = -(2-3.6*y(idx).^6/R^6);
Delta_alpha_rel(~idx) = R^2./y(~idx).^2 * 1.6;

% Delta_alpha_rel(idx) = -(1-2*y(idx).^2/R^2);
% Delta_alpha_rel(~idx) = R^2./y(~idx).^2;

end