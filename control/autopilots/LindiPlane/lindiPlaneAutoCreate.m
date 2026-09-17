function [ap,ap_notune] = lindiPlaneAutoCreate( airplane, varargin )
% lindiPlaneAutoCreate automatic computation of LindiPlane autopilot
% parameters
% 
% Syntax:
%   ap = lindiPlaneAutoCreate( copter )
%   ap = lindiPlaneAutoCreate( copter, Name, Value )
% 
% Inputs:
%   airplane        Airplane parameters struct, see conventionalAirplaneLoadParams
%   Name            Name of Name-Value arguments:
%                       - 'GyroFilt': define gyro low-pass filter (inner
%                           control loop)
%                           Value: [omega,d], where omega is the cutoff
%                           frequency (rad/s) and d is the damping ratio
%                           (0...1), default: omega = servo cutoff
%                           frequency, d = 1
%                       - 'AccFilt': define accelerometer low-pass filter
%                           (outer control loop)
%                           Value: [omega,d], where omega is the cutoff
%                           frequency (rad/s) and d is the damping ratio
%                           (0...1), default: omega = 2 * gyro filter 
%                           cutoff frequency, d = 1
%                       - 'AgilityAtti': define agility of attitude
%                           controller with values between 0.5 ... 2
%                           (0.5: 50% agility, 1: 100% agility, 2: 200%
%                           agility), default: 1
%                       - 'AgilityPos': define agility of position
%                           controller with values between 0.5 ... 2
%                           (0.5: 50% agility, 1: 100% agility, 2: 200%
%                           agility), default: 1
%                       - 'ServoBoost': define servo booster strength
%                           with values between 0.5 ... 2 (0.5: 50% servo
%                           speed, 1: 100% servo speed, 2: 200% servo
%                           speed), default: 1
%                       - 'MlaUse': define whether maneuver load alleviation
%                           should be used (0: off, 1: on), default: 0
%                       - 'Uaero': define how unsteady aerodynamics
%                           should be considered (1x1 to 1x4 array)
%                           - 1st element: Should unsteady flap 
%                               aerodynamics be considered as part of the
%                               actuator dynamics? (0: no, 1: yes),
%                               default: 0
%                           - 2nd element: Should the slow part of the
%                               transfer function be boosted/eliminated?
%                               (0: no, 1: boosted to Vref (4th element),
%                               2: eliminated), default: 0
%                           - 3rd element: Should the fast part of the
%                               transfer function be boosted/eliminated?
%                               (0: no, 1: boosted to Vref (4th element),
%                               2: eliminated), default: 0
%                           - 4th element: Reference airspeed Vref to which
%                               the unsteady flap aerodynamics should be
%                               boosted (empty: computed internally), m/s
%                       - 'UaeroDlc': define how unsteady aerodynamics
%                           should be considered for DLC (1x1 to 1x4 array)
%                           (see 'Uaero')
%   Value           Value of Name-Value arguments (see input Name)
% 
% Outputs:
%   ap              LindiPlane autopilot parameters struct as defined by
%                   this function (all parameters are tunable)
%   ap_notune       Non-tunable LindiPlane autopilot parameters struct as
%                   defined by this function

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Input parsing
gyro_flt_default            = [0,1];
acc_flt_default             = [0,1];
agility_atti                = 1;
agility_pos                 = 1;
servo_boost                 = 1;
mla_use                     = 0;
uaero_config                = zeros(1,4);
uaero_config_dlc            = zeros(1,4);

p = inputParser;
addParameter(p, 'GyroFilt',    gyro_flt_default,@(x) numel(x)==2);
addParameter(p, 'AccFilt',     acc_flt_default,@(x) numel(x)==2);
addParameter(p, 'AgilityAtti', agility_atti);
addParameter(p, 'AgilityPos',  agility_pos);
addParameter(p, 'ServoBoost',  servo_boost);
addParameter(p, 'MlaUse',      mla_use);
addParameter(p, 'Uaero',       uaero_config);
addParameter(p, 'UaeroDlc',    uaero_config_dlc);

parse(p,varargin{:});

gyro_flt = p.Results.GyroFilt;
acc_flt = p.Results.AccFilt;
agility_atti = p.Results.AgilityAtti;
agility_pos = p.Results.AgilityPos;
servo_boost = p.Results.ServoBoost;
mla_use = p.Results.MlaUse;
uaero_config = p.Results.Uaero;
uaero_config_dlc = p.Results.UaeroDlc;


cntrl_effect_scaling_factor = 1;


%% Flap control effectiveness
cef.clu = [];
cef.s = [];
cef.rotx = [];
cef.x = [];
cef.y = [];
cef.z = [];
fdepth = [];
c = [];
aero_names = fieldnames(airplane.aero);
for i = 1:length(aero_names)
    if contains(aero_names{i},'wing')
        wing = airplane.aero.(aero_names{i});
        num_flaps = length(wing.flap.dalpha_deta);
        pos_long = [wing.flap.x_cp0_wing;zeros(2,num_flaps)];
        pos_span = [zeros(1,num_flaps);wing.flap.y_cp_wing;zeros(1,num_flaps)];
        if contains(aero_names{i},'Main')
            cla = wing.polar.params.C_Lalpha*ones(1,num_flaps);
            dadf = wing.flap.dalpha_deta/2;
            dfdu = airplane.act.ailerons.deflectionMax;
            cef.clu(end+1:end+num_flaps) = cla .* dadf .* dfdu;
            pos = airplane.aero.config.wingMainPos + pos_span + pos_long - airplane.config.cg;
            cef.x(end+1:end+num_flaps) = pos(1,:);
            cef.y(end+1:end+num_flaps) = pos(2,:);
            cef.z(end+1:end+num_flaps) = pos(3,:);
            cef.rotx(end+1:end+num_flaps) = 0;
            cef.s(end+1:end+num_flaps) = wing.geometry.S;
            c(end+1:end+num_flaps) = wing.geometry.S/wing.geometry.b;
            fdepth(end+1:end+num_flaps) = wing.flap.lambda_K;
            cl_max = wing.polar.params.C_Lmax;
            cl_min = -wing.polar.params.C_Lmin;
        elseif contains(aero_names{i},'Htp')
            cla_h = wing.polar.params.C_Lalpha;
            dadf = mean(wing.flap.dalpha_deta);
            dfdu = airplane.act.elevator.deflectionMax;
            cef.clu(end+1) = cla_h .* dadf .* dfdu;
            pos = airplane.aero.config.wingHtpPos + mean(pos_span,2) + mean(pos_long,2) - airplane.config.cg;
            cef.x(end+1) = pos(1,:);
            cef.y(end+1) = pos(2,:);
            cef.z(end+1) = pos(3,:);
            cef.rotx(end+1) = 0;
            cef.s(end+1) = wing.geometry.S;
            c(end+1) = wing.geometry.S/wing.geometry.b;
            fdepth(end+1) = wing.flap.lambda_K;
        elseif contains(aero_names{i},'Vtp')
            cla_v = wing.polar.params.C_Lalpha;
            dadf = mean(wing.flap.dalpha_deta);
            dfdu = airplane.act.rudder.deflectionMax;
            cef.clu(end+1) = cla_v .* dadf .* dfdu;
            pos = airplane.aero.config.wingVtpPos + mean(pos_span,2) + mean(pos_long,2) - airplane.config.cg;
            cef.x(end+1) = pos(1,:);
            cef.y(end+1) = pos(2,:);
            cef.z(end+1) = pos(3,:);
            euler_angles = dcm2Euler(airplane.aero.config.wingVtpRot);
            cef.rotx(end+1) = euler_angles(1);
            cef.s(end+1) = wing.geometry.S;
            c(end+1) = wing.geometry.S/wing.geometry.b;
            fdepth(end+1) = wing.flap.lambda_K;
        end
    end
end
ap.cef = cef;



%% Body control effectiveness
ap.ceb.m    = cntrl_effect_scaling_factor * airplane.body.m;
ap.ceb.ixx  = cntrl_effect_scaling_factor * airplane.body.I(1,1);
ap.ceb.iyy  = cntrl_effect_scaling_factor * airplane.body.I(2,2);
ap.ceb.izz  = cntrl_effect_scaling_factor * airplane.body.I(3,3);
ap.ceb.ixy  = 0 * cntrl_effect_scaling_factor * -airplane.body.I(1,2);
ap.ceb.ixz  = 0 * cntrl_effect_scaling_factor * -airplane.body.I(1,3);
ap.ceb.iyz  = 0 * cntrl_effect_scaling_factor * -airplane.body.I(2,3);

% control effectiveness scaling
ap.ceb.scale = 1;


%% Servos
ap.servo.omega  = airplane.act.ailerons.naturalFrequency;
ap.servo.d      = airplane.act.ailerons.dampingRatio;
ap.servo.delay  = airplane.act.ailerons.delay;
ap.servo.boost  = servo_boost;


%% Sensor filter
if gyro_flt(1) == 0
    ap.atc.flt.omega = ap.servo.omega * ap.servo.boost;
    ap.atc.flt.d = 1;
else
    ap.atc.flt.omega = gyro_flt(1);
    ap.atc.flt.d = gyro_flt(2);
end
% select if a single (1) or a duplicate (2) low-pass filter should be used
% for the gyros
ap.atc.flt.numGyrFlt = 2;

if acc_flt(1) == 0
    ap.psc.flt.omega = 0.5 * ap.atc.flt.omega;
    ap.psc.flt.d = 1;
else
    ap.psc.flt.omega = acc_flt(1);
    ap.psc.flt.d = acc_flt(2);
end


%% Airspeed
ap.aspd.flttc = 0.1;
ap.aspd.stall = sqrt( airplane.body.m*9.81 / (0.5*1.225*cl_max*airplane.aero.wingMain.geometry.S) );
ap.aspd.min = 1/3 * ap.aspd.stall;


%% Unsteady aerodynamics
if length(uaero_config) < 4
    uaero_config = [uaero_config,zeros(1,4-length(uaero_config))];
end
if uaero_config(4) < 1
    uaero_config(4) = 2 * ap.aspd.stall;
end
ap.uaero.use = uaero_config(1);
ap.uaero.opt1 = uaero_config(2);
ap.uaero.opt2 = uaero_config(3);
ap.uaero.c = c;
ap.uaero.fdepth = fdepth;
ap.uaero.Vref = uaero_config(4);
ap.uaero.cref = mean(c);
ap.uaero.fdref = mean(fdepth);
% rough approximation of time delay due to unsteady flap aerodynamics
if ap.uaero.use == 1 && ap.uaero.opt1 < 2
    T_uaero = ap.uaero.cref/(2*ap.uaero.Vref*0.3);
else
    T_uaero = 0;
end

%% Attitude control
T_h = 2/(ap.servo.omega*ap.servo.boost) + ap.servo.delay + 2/ap.atc.flt.omega + T_uaero;
% p = -1*[1,1,1] * 2/T_h / 6 * agility_atti;
% p = -1*[1,0.9+0.7*1i,0.9-0.7*1i] * 2/T_h / 5.6 * agility_atti;
p = roots([1,6,15,15]) * 2/T_h / 12 * agility_atti;
k = ndiFeedbackGainPlace( p, T_h );

p2 = -1*[1,1] * 2/T_h / 4;
% p2 = roots([1,3,3]) * 2/T_h / 6 * agility_atti;
k2 = ndiFeedbackGainPlace( p2, T_h );

ap.atc.k.rang = k(1);
ap.atc.k.rrat = k(2);
ap.atc.k.racc = k(3);

ap.atc.k.pang = k(1);
ap.atc.k.prat = k(2);
ap.atc.k.pacc = k(3);

ap.atc.k.yrat = k2(1);
ap.atc.k.yacc = k2(2);


%
ap.atc.rm.rfreq = 2/T_h / 4 * agility_atti;
ap.atc.rm.rangmax = 70;
ap.atc.rm.rratmax = 70;

ap.atc.rm.pfreq = 1/T_h / 4 * agility_atti;
ap.atc.rm.pangmax = 30;
ap.atc.rm.pratmax = 60;

ap.atc.rm.yfreq = 1/T_h / 4 * agility_atti;
ap.atc.rm.yratmax = 60;
ap.atc.rm.ydecaytc = 2/ap.atc.rm.yfreq;


%% State dynamics
% Roll damping inversion parameters
derivs = simpleWingGetDerivs( airplane.aero.wingMain );
% Roll damping derivative
ap.eig.clp = derivs.P(4);
% Wing span, in m
ap.eig.b = airplane.aero.wingMain.geometry.b;
% Wing area, in m^2
ap.eig.s = airplane.aero.wingMain.geometry.S;

% Pitch damping inversion parameters
% Horizontal tailplane lift curve slope
ap.eig.cla_h = cla_h;
% Horizontal tailplane neutral point x-position, in m
ap.eig.x_h = abs(cef.x(end-1));
% Horizontal tailplane area, in m^2
ap.eig.s_h = cef.s(end-1);

% Pitch stiffness and downwash inversion parameters
% Main wing lift curve slope
ap.eig.cla = cla(1);
% Derivative of horizontal tailplane angle of attack w.r.t. main wing angle
% of attack @alpha_htp/@alpha
ap.eig.dahda = airplane.aero.downwash.alpha_htp_dalpha;
% Derivative of horizontal tailplane angle of attack w.r.t. main wing flap
% commands, in rad
ap.eig.dahdu = airplane.aero.downwash.alpha_htp_deta.*airplane.act.ailerons.deflectionMax;
% Center of gravity x-position, in m
ap.eig.xcg = airplane.config.cg(1);
x_np_main = airplane.aero.config.wingMainPos(1) + airplane.aero.wingMain.xyz_wing_np(1);
x_np_htp = airplane.aero.config.wingHtpPos(1) + airplane.aero.wingHtp.xyz_wing_np(1);
% Airplane neutral point x-position, in m
ap.eig.xnp = ( (1+ap.eig.dahda)*ap.eig.cla_h*ap.eig.s_h*x_np_htp + ap.eig.cla*ap.eig.s*x_np_main ) ...
    / ( (1+ap.eig.dahda)*ap.eig.cla_h*ap.eig.s_h + ap.eig.cla*ap.eig.s );
% Airplane neutral point x-position neglecting downwash, in m
ap.eig.xnp0 = ( ap.eig.cla_h*ap.eig.s_h*x_np_htp + ap.eig.cla*ap.eig.s*x_np_main ) ...
    / ( ap.eig.cla_h*ap.eig.s_h + ap.eig.cla*ap.eig.s );


%% Position control
T_h = T_h + 2/ap.atc.rm.rfreq + 2/ap.psc.flt.omega - 2/ap.atc.flt.omega;
% p = -1*[1,1,1] * 2/T_h / 5;
% p = -1*[1,0.9+0.7*1i,0.9-0.7*1i] * 2/T_h / 5.6 * agility_pos;
p = roots([1,6,15,15]) * 2/T_h / 12 * agility_pos;
k = ndiFeedbackGainPlace( p, T_h );

ap.psc.k.pos = k(1);
ap.psc.k.vel = k(2);
ap.psc.k.acc = k(3);


%% Flight path inversion
% Selects the inversion method used for flight-path control (0: NDI,
% 1: INDI)
ap.fpi.inv_mode = 0;
% Enables limiting of the desired specific force based on the
% maximum/minimum allowed lift coefficient
ap.fpi.cl_lim_use = 1;
% Maximum positive lift coefficient used to limit the desired specific lift
% force
ap.fpi.cl_max = 0.75*cl_max;
% Minimum lift coefficient used to limit the desired specific lift force
% during negative-lift operation
ap.fpi.cl_min = 0.75*cl_min;
% Minimum desired acceleration magnitude required to update the commanded
% roll orientation (m/s^2). Below this value, the previous roll orientation
% is retained.
ap.fpi.rll_acc_min = 3.3;
% Maximum duration during which negative lift is used, in s
ap.fpi.t_flip = 2;

%% Waypoint navigation
% spatial filter constant for flight path smoothing (pre-filtering), in m
ap.wpnav.L_flt = 3 * ap.aspd.stall * 2/ap.atc.rm.rfreq;
% waypoint acceptance radius, in m
ap.wpnav.wprad = 60;
% maximum position error (switch to approach if above)
ap.wpnav.eposmax = 20;

%% Control allocation
num_u = length(ap.cef.clu);
% minimum control input kx1 vector
ap.ca.u_min = -ones(num_u,1);
% maximum control input kx1 vector
ap.ca.u_max = ones(num_u,1);
% desired control input kx1 vector
ap.ca.u_d = zeros(num_u,1);

% weighting mx1 vector of pseudo-control
ap.ca.W_v = [ 1; 1; 1 ; 1 ];
% weighting kx1 vector of the control input vector
ap.ca.W_u = ones(num_u,1);
ap.ca.W_u(end-1) = 0.1;
% weighting of pseudo-control vs. control input (scalar)
ap.ca.gamma = 1000;
% initial working set mx1 vector
ap.ca.W = zeros(num_u,1);
% maximum number of iterations (scalar)
ap.ca.i_max = 1;


%% Direct lift control
% 0: disabled, 1: compensate elevator force, 2: acc feedback control, 3:
% acc feedback control with filter ap.dlc.flt
ap.dlc.opt = 1;
% collective aileron decay, in m/s^2 / 1
ap.dlc.flapdecay = 4;
% cutoff pitch angle error, in deg
ap.dlc.maxptch = 20;
% acceleration filter cutoff frequency, in rad/s
ap.dlc.flt.omega = ap.atc.flt.omega;
% acceleration filter damping ratio
ap.dlc.flt.d = ap.atc.flt.d;
% servo boost
ap.dlc.srv.boost = ap.servo.boost;
% unsteady flap aerodynamics
if length(uaero_config_dlc) < 4 || isequal(uaero_config_dlc,zeros(1,4))
    uaero_config_dlc = uaero_config;
end
if uaero_config_dlc(4) < 1
    uaero_config_dlc(4) = 2 * ap.aspd.stall;
end
ap.dlc.ua.use = uaero_config_dlc(1);
ap.dlc.ua.opt1 = uaero_config_dlc(2);
ap.dlc.ua.opt2 = uaero_config_dlc(3);
ap.dlc.ua.Vref = uaero_config_dlc(4);


%% Maneuver load alleviation
ap.mla.use = mla_use;
ap.mla.eta_np = airplane.aero.wingMain.xyz_wing_np(2,:)/(0.5*airplane.aero.wingMain.geometry.b);
ap.mla.ca.W_v = [ 1; 1; 10; 10 ];
ap.mla.ca.W_u = ap.ca.W_u;
ap.mla.ca.gamma = ap.ca.gamma;
ap.mla.ca.i_max = ap.ca.i_max;


%% Sample time

ap_notune.ts = 1/400;

end