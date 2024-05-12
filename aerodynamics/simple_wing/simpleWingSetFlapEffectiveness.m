function wing = simpleWingSetFlapEffectiveness( wing )

% Literature:
%   [1] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

lambda = wing.flap.lambda_K;

if isfield(wing.flap,'eta_0')
    eta = wing.flap.eta_0;
else
    eta = wing.flap.eta;
end
if length(eta) == 1
    x = wing.flap.x;

    % [1] page 440, Eq. 12.7
    dalpha_deta_x1 = -2/pi * ( sqrt(lambda*(1-lambda)) + asin(sqrt(lambda)) );

    % [1] page 442, Eq. 12.9
    dalpha_deta = -lambda + x * ( dalpha_deta_x1 + lambda );

    % [1] page 450, Eq. 12.25
    dalpha_deta = dalpha_deta * ...
        ( -1 + 2/pi * ( acos(eta) - eta*sqrt(1-eta^2) ) );

    wing.flap.dalpha_deta = dalpha_deta*[1,1];
else
    aspect_ratio = wing.geometry.b^2 / wing.geometry.S;
    prm = wing_parametric( aspect_ratio, wing.geometry.z, wing.geometry.phi, 0, ...
        'Flaps', eta, lambda );
    vlm_wing = wingCreate( prm, 40, 'spacing', 'constant' );
    derivs = wingGetDerivs( vlm_wing );
end

end
