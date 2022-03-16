function horiz_pos_cntrl = fmCopterLoiterIndiLoadParams( filename )
% fmCopterLoiterIndiLoadParams loads a reduced attitude controller
%   struct
% 
% Example:
%  horiz_pos_cntrl = fmCopterLoiterIndiLoadParams( ...
%   'fm_copter_loiter_indi_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);


%% limits

% maximum lateral position error, in m
horiz_pos_cntrl.limits.e_xy_max = diag(horiz_pos_cntrl.K(:,3:4))*horiz_pos_cntrl.rm.uv_max*0.7./diag(horiz_pos_cntrl.K(:,1:2));
% maximum commanded lateral acceleration by position controller, in m/s^2
horiz_pos_cntrl.limits.uv_dt_max = 0.75 * horiz_pos_cntrl.rm.uv_dt_max;

end