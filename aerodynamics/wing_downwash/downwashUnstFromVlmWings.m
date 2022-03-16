function dw = downwashUnstFromVlmWings( wing_main, wing_htp, V )
% downwashFromVlmWings compute the coefficients of a transfer function for
% unsteady downwash of a main wing to a horizontal tailplane.
% 
% Inputs:
%   wing_main     	wing struct (see wingCreate) for the main wing
%   wing_htp     	wing struct (see wingCreate) for the horizontal
%                   tailplane
%   V               airspeed (scalar); can be scheduled so that the
%                   transfer function does not need to be completely
%                   recomputed for other airspeeds (see
%                   downwashUnstSchedule)
% Outputs:
%   dw              struct with the following fields:
%                       - b:    numerator coefficients of the transfer
%                               function
%                       - a:    denominator coefficients of the transfer
%                               function
%                       - V:    airspeed (same as input of this function)
%                       - l:    horizontal distance between wings
% 
% See also:
%   downwashUnstAnalytic, downwashUnstSchedule
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% mean aerodynamic center (c/4) and mean trailing edge position of main
% wing in the span section of the horizontal tailplane
idx_ctrl = wing_main.geometry.ctrl_pt.pos(2,:) > -wing_htp.params.b/2 & ...
    wing_main.geometry.ctrl_pt.pos(2,:) < wing_htp.params.b/2;
idx_vortex = wing_main.geometry.vortex.pos(2,:) > -wing_htp.params.b/2 & ...
    wing_main.geometry.vortex.pos(2,:) < wing_htp.params.b/2;
wing_pos_trail = mean(wing_main.geometry.ctrl_pt.pos(:,idx_ctrl),2) - [mean(wing_main.geometry.ctrl_pt.c(idx_ctrl),2)/4;0;0] + wing_main.geometry.origin;
wing_pos_vortex = mean(wing_main.geometry.vortex.pos(:,idx_vortex),2) + wing_main.geometry.origin;

% mean leading edge position of horizontal tailplane
htp_pos_lead = mean(wing_htp.geometry.vortex.pos,2) + [mean(wing_htp.geometry.ctrl_pt.c,2)/4;0;0] + wing_htp.geometry.origin;

% parameters of the wingDownwashAnalytic function
l = abs( wing_pos_vortex(1) - htp_pos_lead(1) );
l_1 = abs( wing_pos_trail(1) - htp_pos_lead(1) );
z = wing_pos_trail(3) - htp_pos_lead(3);

% generate time domain downwash data
time_data = 0:0.01:2;
step_response_data = downwashUnstAnalytic(V,l,l_1,z,time_data,0);
data=iddata([zeros(1,40),step_response_data]',[zeros(1,40),ones(1,length(time_data))]',0.01);

% fit transfer function to time domain data (System Identification Toolbox
% required)
sys=tfest(data,13,13);

% numerator and denominator coefficients of the transfer function
b = sys.Numerator;
a = sys.Denominator;

% create output struct
dw.b = b;
dw.a = a;
dw.V = V;
dw.l = l;

% plot result
dw_tf = tf(dw.b,dw.a);
plot(time_data,step_response_data,'r-','LineWidth',2)
hold on
step(dw_tf)
legend('time domain data','fitted transfer function')

end