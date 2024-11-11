function derivs = wingGetDownwashDerivs( wing_main, wing_htp )
% wingGetDownwashDerivs computes the downwash derivatives for a
%   wing-HTP-combinatione using lifting line theory (HTP: horizontal
%   tailplane)
% 
% Syntax:
%   derivs = wingGetDownwashDerivs( wing_main, wing_htp )
% 
% Inputs:
%   wing_main           Main wing struct (see wingCreate)
%   wing_htp            Horizontal tailplane wing struct (see wingCreate)
% 
% Outputs:
%   derivs.alpha_htp_dalpha Mean induced angle of attack gradients at the
%                           HTP per angle of attack of the main wing
%                           @alpha_htp/@alpha (scalar), non-dimensional
%   derivs.alpha_htp_deta 	Mean induced angle of attack gradients at the
%                           HTP per main wing flap deflection angles
%                           @alpha_htp/@eta (1xM matrix for M flaps),
%                           non-dimensional
% 
% See also:
%   wingCreate, wingGetDownwash, vlmVoringsLlt

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

wing_origin = wing_main.geometry.origin;
htp_origin = wing_htp.geometry.origin;
b = wing_htp.params.b;
% Set minimum vertical distance between main wing and HTP to avoid
% singularities
diff_vertical = htp_origin(3) - wing_origin(3);
if abs( diff_vertical ) < 0.1 * b
    htp_origin(3) = wing_origin(3) - sign(diff_vertical) * 0.1 * b;
elseif diff_vertical == 0
    htp_origin(3) = wing_origin(3) - 0.1 * b;
end
wing_htp.geometry.origin = htp_origin;

htp_pos = wing_htp.geometry.ctrl_pt.pos + wing_htp.geometry.origin;

[alpha_htp_dalpha,alpha_htp_deta] = wingGetDownwash( wing_main, htp_pos );

c = wing_htp.geometry.ctrl_pt.c;
dy = diff(wing_htp.geometry.vortex.pos(2,:,1));
s = sum(c.*dy);
AIC_norm = wing_htp.interim_results.AIC_b+wing_htp.interim_results.AIC_t;

alpha_eff_dalpha = b*-1/pi*inv(diag(c)*AIC_norm);

htp_derivs = wingGetDerivs(wing_htp);

derivs.alpha_htp_dalpha = -2*pi/htp_derivs.alpha(3) * sum( (alpha_eff_dalpha*alpha_htp_dalpha')'.*c.*dy/s, 2 )';
derivs.alpha_htp_deta = -2*pi/htp_derivs.alpha(3) * sum( (alpha_eff_dalpha*alpha_htp_deta')'.*c.*dy/s, 2 )';

end