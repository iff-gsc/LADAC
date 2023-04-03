function ap = lindiCopterAutoCreate( copter, varargin )
% lindiCopterAutoCreate automatic computation of LindiCopter autopilot
% parameters
% 
% Syntax:
%   ap = lindiCopterAutoCreate( copter )
%   ap = lindiCopterAutoCreate( copter, Name, Value )
% 
% Inputs:
%   copter          multicopter parameters struct, see copterLoadParams
%   Name            name of Name-Value Arguments:
%                       - 'AgilityAtti': define agility of attitude
%                           controller with values between -1 ... 1
%                           (-1: low, 0: medium, 1: high), default: 0
%                       - 'AgilityPos': define agility of position
%                           controller with values between -1 ... 1
%                           (-1: low, 0: medium, 1: high), default: 0
%                       - 'FilterStrength': define sensor low-pass filter
%                           strength with values between -1 ... 1 (-1:
%                           weak, 0: medium, 1: strong), default: 0
%                       - 'CntrlEffectScaling': define scaling of control
%                           effectiveness with values between 0.75 ... 1
%                           (0.75: reduce to 75%, 1: no scaling),
%                           default: 1
%   Value           value of Name-Value Arguments (see input Name)
% 
% Outputs:
%   ap              LindiCopter autopilot parameters struct as defined by
%                   this function

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% initialize default tuning parameters
agility_atti                = 0;
agility_pos                 = 0;
filter_strength             = 0;
cntrl_effect_scaling_factor = 1;

% parse name-value arguments
for i = 1:length(varargin)
    if ischar(varargin{i})
        if isequal(varargin{i},'AgilityAtti')
            agility_atti(:) = varargin{i+1};
        elseif isequal(varargin{i},'AgilityPos')
            agility_pos(:) = varargin{i+1};
        elseif isequal(varargin{i},'FilterStrength')
            filter_strength(:) = varargin{i+1};
        elseif isequal(varargin{i},'CntrlEffectScaling')
            cntrl_effect_scaling_factor(:) = varargin{i+1};
        end
    end
end

% assure tuning parameter limits
agility_atti(:) = max( min(agility_atti,1), -1 );
filter_strength(:) = max( min(filter_strength,1), -1 );
ce_scaling_max = 1;
ce_scaling_min = 0.75;
cntrl_effect_scaling_factor(:) = max( ...
    min(cntrl_effect_scaling_factor,ce_scaling_max), ce_scaling_min );

% aggressiveness ( aggr_min ... 1 )
aggr_min = 0.5;
aggr_atti = 1 - 0.5*(1-aggr_min)*(1-agility_atti);
aggr_pos = 1 - 0.5*(1-aggr_min)*(1-agility_pos);

% filter cutoff factor ( 1/4 ... 4 )
filter_factor = 4^(-filter_strength);

% time scale separation factor (2-5)
sep_factor_min = 2;
sep_factor_max = 5;
sep_factor_atti = sep_factor_min + (1-aggr_atti)/(1-aggr_min)*(sep_factor_max-sep_factor_min);
sep_factor_pos = sep_factor_min + (1-aggr_pos)/(1-aggr_min)*(sep_factor_max-sep_factor_min);



is_flipped_allowed = 1;

g = 9.81;

ap.ca = loadParams( 'caWls_params_default' );
ap.ca.W_v(1,1) = 30;
ap.ca.W_v(2,2) = 30;
ap.ca.W_v(3,3) = 0.01;
ap.ca.W_v(4,4) = 300;

% maximum number of waypoints
ap.traj.wpmax = 4;
% boolean if last and first waypoint should be connected (true) or not
ap.traj.cycle = true;
% degree of spline polynomials
ap.traj.degree = 5;

num_motors = size(copter.config.propPos_c,2);

[k,d] = propMapFitGetFactors(copter.prop.map_fit);
omega_max = motorStaticSpeed(copter.motor.KT,copter.motor.R,copter.bat.V,d,ap.ca.u_max);
omega_min = motorStaticSpeed(copter.motor.KT,copter.motor.R,copter.bat.V,d,ap.ca.u_min);


thrust_max = sum( k*omega_max.^2 );
thrust_min = sum( k*omega_min.^2 );
thrust_hover = copter.body.m * g;

omega_hover = sqrt( thrust_hover / k );

delta_thrust_max = min( thrust_max - thrust_hover, thrust_hover - thrust_min );

omega_delta_max = sqrt( ( thrust_hover + delta_thrust_max ) / k );
omega_delta_min = sqrt( ( thrust_hover - delta_thrust_max ) / k );

torque_delta_max = d * omega_delta_max^2;
torque_delta_min = d * omega_delta_min^2;

acc_up_max = thrust_max / copter.body.m;
acc_down_max = thrust_min / copter.body.m;

ap.psc.rm.accumax = aggr_pos * ( acc_up_max - g );
ap.psc.rm.accdmax = aggr_pos * ( g - acc_down_max );

prop_name_cell = strsplit(copter.prop.name,'x');
prop_diameter = str2num(prop_name_cell{1}) * 2.54/100;
A = num_motors * pi/4 * prop_diameter^2;
[ ~, ~, v_i0 ] = inducedVelocityWithUnitsAngleOfAttack( 0, -pi/2, thrust_hover, 1.225, A );


% temp: to do
ap.psc.rm.veldmax = aggr_pos * v_i0;


omega = 0;
V = 0;
dt = 0.01;
while true
    u = aggr_atti;
    torque = propMapFitGetZ(copter.prop.map_fit,omega*60/(2*pi),V,'torque');
    thrust = propMapFitGetZ(copter.prop.map_fit,omega*60/(2*pi),V,'thrust');
    dot_omega = copter.motor.KT/copter.motor.R/copter.prop.I*(copter.bat.V*u-copter.motor.KT*omega)-torque/copter.prop.I;
    acc = 1/copter.body.m * num_motors*thrust - 1.225*copter.aero.S*copter.aero.C_Dmax*V^2;
    omega = omega + dot_omega*dt;
    V = V + acc*dt;
    if abs(dot_omega) < 0.1 && abs(acc) < 0.01
        break;
    end
end

ap.psc.rm.velumax = aggr_pos * V;
ap.psc.rm.velxymax = aggr_pos * V;

lean_max = acos( g / acc_up_max );
if isreal(lean_max)
    acc_xy_max = acc_up_max * sin( lean_max );
else
    error('Not enough thrust to hover.')
end
if is_flipped_allowed
    ap.atc.rm.leanmax = pi;
else
    ap.atc.rm.leanmax = lean_max;
end
ap.atc.rm.leandamp = 1;
ap.atc.rm.yawratemax = 2*pi;

ap.psc.rm.accxymax = aggr_pos * acc_xy_max;
ap.psc.rm.veltc = 1.25/aggr_pos * ap.psc.rm.velxymax / ap.psc.rm.accxymax;



M = zeros(3,num_motors);
for i=1:num_motors
    M(:,i) = evalin('caller',['copter.config.M_b_prop',num2str(i)]) * [1;0;0];
end

% propeller control effectiveness parameters
ap.cep.k 	= k;
ap.cep.d	= d;
ap.cep.x	= copter.config.propPos_c(1,:) - copter.config.CoG_Pos_c(1);
ap.cep.y	= copter.config.propPos_c(2,:) - copter.config.CoG_Pos_c(2);
ap.cep.z	= copter.config.propPos_c(3,:) - copter.config.CoG_Pos_c(3);
ap.cep.a	= copter.config.propDir(:)';
ap.cep.nx	= M(1,:);
ap.cep.ny	= M(2,:);
ap.cep.ip	= copter.prop.I;
ap.cep.kt	= copter.motor.KT;
ap.cep.vb	= copter.bat.V;
ap.cep.ri	= copter.motor.R;

% body control effectiveness parameters
ap.ceb.m    = cntrl_effect_scaling_factor * copter.body.m;
ap.ceb.ixx  = cntrl_effect_scaling_factor * copter.body.I(1,1);
ap.ceb.iyy  = cntrl_effect_scaling_factor * copter.body.I(2,2);
ap.ceb.izz  = cntrl_effect_scaling_factor * copter.body.I(3,3);
ap.ceb.ixy  = cntrl_effect_scaling_factor * -copter.body.I(1,2);
ap.ceb.ixz  = cntrl_effect_scaling_factor * -copter.body.I(1,3);
ap.ceb.iyz  = cntrl_effect_scaling_factor * -copter.body.I(2,3);



L = cross( copter.config.propPos_c, M );

thrust_vector = zeros( num_motors, 1 );
torque_vector = zeros( num_motors, 1 );


thrust_vector(L(1,:)'>0) = ( thrust_hover + delta_thrust_max ) / num_motors;
thrust_vector(L(1,:)'<0) = ( thrust_hover - delta_thrust_max ) / num_motors;
roll_moment_max = L(1,:) * thrust_vector;

acc_roll_max = roll_moment_max / copter.body.I(1,1);

thrust_vector(L(2,:)'>0) = ( thrust_hover + delta_thrust_max ) / num_motors;
thrust_vector(L(2,:)'<0) = ( thrust_hover - delta_thrust_max ) / num_motors;
pitch_moment_max = L(2,:) * thrust_vector;

acc_pitch_max = pitch_moment_max / copter.body.I(2,2);

acc_roll_pitch_max = min( acc_roll_max, acc_pitch_max );


torque_vector(ap.cep.a>0) = torque_delta_max/num_motors;
torque_vector(ap.cep.a<0) = -torque_delta_min/num_motors;
yaw_moment_max = sum( torque_vector );

acc_yaw_max1 = yaw_moment_max / copter.body.I(3,3);
Delta_u_yaw = 0.1;
acc_yaw_max2 = Delta_u_yaw * copter.motor.KT/copter.motor.R/copter.prop.I*copter.bat.V * num_motors * copter.prop.I/copter.body.I(3,3);
acc_yaw_max = mean([acc_yaw_max1,acc_yaw_max2]);

leanfreq_force = 2*pi*sqrt( 0.25*acc_roll_pitch_max / lean_max );
yawratetc_force = ap.atc.rm.yawratemax / acc_yaw_max;

ap.mtc = copter.motor.R*copter.prop.I/copter.motor.KT^2;

ap.ts = 0.0025;

% sensor filter
ap.sflt.D = 1;
ap.sflt.omega = filter_factor*2/ap.mtc;

ap.mtc = 1.2*ap.mtc;

T_h_1 = ap.mtc + 2/ap.sflt.omega;

leanfreq_sep = 1/sep_factor_atti*2/( T_h_1 );
yawratetc_sep = sep_factor_atti*( T_h_1 );
ap.atc.rm.leanfreq = min( leanfreq_force, leanfreq_sep );
ap.atc.rm.yawratetc = max( yawratetc_force, yawratetc_sep );

% lean angle controller
% max_roll_pitch_atti_error = 2;
% k = maxError2FeedbackGain( max_roll_pitch_atti_error, acc_roll_pitch_max, T_h, aggr );
k = ndiFeedbackGainPlace(-ap.atc.rm.leanfreq*[1,1,1],T_h_1);
ap.atc.k.lean = k(1);
ap.atc.k.leanrate = k(2);
ap.atc.k.leanacc = k(3);

% yaw controller
% max_yaw_atti_error = pi;
% k = maxError2FeedbackGain( max_yaw_atti_error, acc_yaw_max, T_h, aggr );
k = ndiFeedbackGainPlace(-1/ap.atc.rm.yawratetc*[1,1,1],T_h_1);
ap.atc.k.yaw = k(1);
ap.atc.k.yawrate = k(2);
ap.atc.k.yawacc = k(3);

% position feedback controller
T_h_pos = T_h_1 + 2/ap.atc.rm.leanfreq;
% p = -0.5*[1+1i,1-1i,4] * aggr / T_h;
% p = -0.5*[1,1,1] * aggr / T_h;
p = -1.3/sep_factor_pos * [1.7,1+0.65i,1-0.65i] / T_h_pos;
% p = -sep_factor*aggr*[ 1/T_h, 0.7/T_h*(1+0.65i), 0.7/T_h*(1-0.65i) ];
% p = -aggr/sep_factor*[ 1, 1, 1 ]*2/T_h;
k = ndiFeedbackGainPlace(p,T_h_pos);
ap.psc.k.pos = k(1);
ap.psc.k.vel = k(2);
ap.psc.k.acc = k(3);

end
