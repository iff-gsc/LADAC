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
%                       - 'SensFilt': define sensor low-pass filter
%                           Value: [omega,d], where omega is the cutoff
%                           frequency (rad/s) and d is the damping ratio
%                           (0...1), default: omega = twice the servo
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

sflt_default = [];
agility_atti                = 1;
agility_pos                 = 1;
servo_boost                 = 1;

p = inputParser;
addOptional(p,'SensFilt',sflt_default,@(x) numel(x)==2);
addOptional(p,'AgilityAtti',agility_atti);
addOptional(p,'AgilityPos',agility_pos);
addOptional(p,'ServoBoost',servo_boost);

parse(p,varargin{:});

sflt = p.Results.SensFilt;
agility_atti = p.Results.AgilityAtti;
agility_pos = p.Results.AgilityPos;
servo_boost = p.Results.ServoBoost;


cntrl_effect_scaling_factor = 1;


% flap control effectiveness parameters
cef.cla = [];
cef.dadf = [];
cef.dfdu = [];
cef.s = [];
cef.rotx = [];
cef.x = [];
cef.y = [];
cef.z = [];
aero_names = fieldnames(airplane.aero);
for i = 1:length(aero_names)
    if contains(aero_names{i},'wing')
        wing = airplane.aero.(aero_names{i});
        num_flaps = length(wing.flap.dalpha_deta);
        pos_long = [wing.flap.x_cp0_wing;zeros(2,num_flaps)];
        pos_span = [zeros(1,num_flaps);wing.flap.y_cp_wing;zeros(1,num_flaps)];
        if contains(aero_names{i},'Main')
            cef.cla(end+1:end+num_flaps) = wing.polar.params.C_Lalpha*ones(1,num_flaps);
            cef.dadf(end+1:end+num_flaps) = wing.flap.dalpha_deta;
            pos = airplane.aero.config.wingMainPos + pos_span + pos_long - airplane.config.cg;
            cef.x(end+1:end+num_flaps) = pos(1,:);
            cef.y(end+1:end+num_flaps) = pos(2,:);
            cef.z(end+1:end+num_flaps) = pos(3,:);
            cef.rotx(end+1:end+num_flaps) = 0;
            cef.dfdu(end+1:end+num_flaps) = airplane.act.ailerons.deflectionMax;
            cef.s(end+1:end+num_flaps) = wing.geometry.S/2;
        elseif contains(aero_names{i},'Htp')
            cef.cla(end+1) = wing.polar.params.C_Lalpha;
            cef.dadf(end+1) = mean(wing.flap.dalpha_deta);
            pos = airplane.aero.config.wingHtpPos + mean(pos_span,2) + mean(pos_long,2) - airplane.config.cg;
            cef.x(end+1) = pos(1,:);
            cef.y(end+1) = pos(2,:);
            cef.z(end+1) = pos(3,:);
            cef.rotx(end+1) = 0;
            cef.dfdu(end+1) = airplane.act.elevator.deflectionMax;
            cef.s(end+1) = wing.geometry.S;
        elseif contains(aero_names{i},'Vtp')
            cef.cla(end+1) = wing.polar.params.C_Lalpha;
            cef.dadf(end+1) = mean(wing.flap.dalpha_deta);
            pos = airplane.aero.config.wingVtpPos + mean(pos_span,2) + mean(pos_long,2) - airplane.config.cg;
            cef.x(end+1) = pos(1,:);
            cef.y(end+1) = pos(2,:);
            cef.z(end+1) = pos(3,:);
            euler_angles = dcm2Euler(airplane.aero.config.wingVtpRot);
            cef.rotx(end+1) = euler_angles(1);
            cef.dfdu(end+1) = airplane.act.rudder.deflectionMax;
            cef.s(end+1) = wing.geometry.S;
        end
    end
end
num_flaps = size(cef.cla,2);
cef.m = zeros(1,num_flaps);
cef.xm = zeros(1,num_flaps);
ap.cef = cef;



% body control effectiveness parameters
ap.ceb.m    = cntrl_effect_scaling_factor * airplane.body.m;
ap.ceb.ixx  = cntrl_effect_scaling_factor * airplane.body.I(1,1);
ap.ceb.iyy  = cntrl_effect_scaling_factor * airplane.body.I(2,2);
ap.ceb.izz  = cntrl_effect_scaling_factor * airplane.body.I(3,3);
ap.ceb.ixy  = cntrl_effect_scaling_factor * -airplane.body.I(1,2);
ap.ceb.ixz  = cntrl_effect_scaling_factor * -airplane.body.I(1,3);
ap.ceb.iyz  = cntrl_effect_scaling_factor * -airplane.body.I(2,3);

% control effectiveness scaling
ap.ceb.scale = 1;

%
ap.servo.omega = airplane.act.ailerons.naturalFrequency;
ap.servo.boost = servo_boost;



%
if isempty(sflt)
    ap.sflt.omega = 2 * ap.servo.omega * ap.servo.boost;
    ap.sflt.d = 1;
else
    ap.sflt.omega = sflt(1);
    ap.sflt.d = sflt(2);
end

ap.aspd.flttc = 0.1;
ap.aspd.min = sqrt( airplane.body.m*9.81 / (0.5*1.225*airplane.aero.wingMain.polar.params.C_Lmax*airplane.aero.wingMain.geometry.S) );


%%
T_h = 2/(ap.servo.omega*ap.servo.boost) + 2/ap.sflt.omega;
% p = -1*[1,1,1] * 2/T_h / 6 * agility_atti;
% p = -1*[1,0.9+0.7*1i,0.9-0.7*1i] * 2/T_h / 5.6 * agility_atti;
p = roots([1,6,15,15]) * 2/T_h / 12 * agility_atti;
k = ndiFeedbackGainPlace( p, T_h );

p2 = -1*[1,1] * 2/T_h / 4;
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
ap.atc.rm.rangmax = 60;
ap.atc.rm.rratmax = 60;

ap.atc.rm.pfreq = 2/T_h / 4 * agility_atti;
ap.atc.rm.pangmax = 30;
ap.atc.rm.pratmax = 60;

ap.atc.rm.yfreq = 2/T_h / 4 * agility_atti;
ap.atc.rm.yratmax = 60;
ap.atc.rm.ydecaytc = 2/ap.atc.rm.yfreq;

% Roll damping inversion parameters
derivs = simpleWingGetDerivs( airplane.aero.wingMain );
ap.atc.rolldamp.clp = derivs.P(4);
ap.atc.rolldamp.b = airplane.aero.wingMain.geometry.b;
ap.atc.rolldamp.S = airplane.aero.wingMain.geometry.S;

%%
T_h = T_h + 2/ap.atc.rm.rfreq;
% p = -1*[1,1,1] * 2/T_h / 5;
% p = -1*[1,0.9+0.7*1i,0.9-0.7*1i] * 2/T_h / 5.6 * agility_pos;
p = roots([1,6,15,15]) * 2/T_h / 12 * agility_pos;
k = ndiFeedbackGainPlace( p, T_h );

ap.psc.k.pos = k(1);
ap.psc.k.vel = k(2);
ap.psc.k.acc = k(3);


% flight path smoothing (pre-filtering) time constant
ap.psc.rm.T = 3 * 2/ap.atc.rm.rfreq;
% waypoint acceptance radius, in m
ap.psc.rm.wprad = 60;
% maximum position error (switch to approach if above)
ap.psc.rm.eposmax = 20;

%%
num_u = length(ap.cef.cla);
% minimum control input kx1 vector
ap.ca.u_min = -ones(num_u,1);
% maximum control input kx1 vector
ap.ca.u_max = ones(num_u,1);
% desired control input kx1 vector
ap.ca.u_d = zeros(num_u,1);

% weighting mx1 vector of pseudo-control
ap.ca.W_v = [10,10,100]';
% weighting kx1 vector of the control input vector
ap.ca.W_u = ones(num_u,1);
% weighting of pseudo-control vs. control input (scalar)
ap.ca.gamma = 1000;
% initial working set mx1 vector
ap.ca.W = zeros(num_u,1);
% maximum number of iterations (scalar)
ap.ca.i_max = 100;


%%

ap_notune.ts = 1/400;

end