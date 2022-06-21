% This test loads airfoil data, performs an analytic fit and visualizes the
% results.
% After that, an analytic approximation of the different fits for multiple
% Mach numbers is performed using a shallow neural network (with NeuN).

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

alpha_deg = -90:0.1:90;
delta = 0:0.2:2;
Mach = [0.15,0.7,0.78]*cos(deg2rad(17));

%% test actuator state
F15_bl = wingAirfoilMapSetSim(wingAirfoilMapLoadSection('F15_bl'));

map_cl = getScellArrAt( F15_bl.map_cl, 1 );
map_cd = getScellArrAt( F15_bl.map_cd, 1 );
map_cm = getScellArrAt( F15_bl.map_cm, 1 );

fclv = zeros(6,length(F15_bl.Mach.data));
fcdv = zeros(6,length(F15_bl.Mach.data));
fcmv = zeros(5,length(F15_bl.Mach.data));
fdclv = zeros(3,length(F15_bl.Mach.data));
fdcdv = zeros(3,length(F15_bl.Mach.data));
fdcmv = zeros(3,length(F15_bl.Mach.data));
for i = 1:length(F15_bl.Mach.data)
    [fclv(:,i),fcdv(:,i),fcmv(:,i)] = airfoilAnalytic0515AlFit( F15_bl.alpha.data, F15_bl.Mach.data(i), reshape(map_cl(:,i,1,2,1),1,[]), reshape(map_cd(:,i,1,2,1),1,[]), reshape(map_cm(:,i,1,2,1),1,[]), 1 );
end

%%

for i = 1:length(F15_bl.Mach.data)
    [fdclv(:,i),fdcdv(:,i),fdcmv(:,i)] = airfoilAnalytic0515DeFit( F15_bl.alpha.data, F15_bl.actuator_2.data, reshape(map_cl(:,i,1,2,:)-map_cl(:,i,1,2,1),1,[]), reshape(map_cd(:,i,1,2,:)-map_cd(:,i,1,2,1),1,[]), reshape(map_cm(:,i,1,2,:)-map_cm(:,i,1,2,1),1,[]), 1 );
end

%%
Maq = [ Mach(1):0.1:Mach(end), Mach(end) ];
fclvq = interp1( F15_bl.Mach.data', fclv', Maq', 'pchip' )';
fcdvq = interp1( F15_bl.Mach.data', fcdv', Maq', 'pchip' )';
fcmvq = interp1( F15_bl.Mach.data', fcmv', Maq', 'pchip' )';
fdclvq = interp1( F15_bl.Mach.data', fdclv', Maq', 'pchip' )';
fdcdvq = interp1( F15_bl.Mach.data', fdcdv', Maq', 'pchip' )';
fdcmvq = interp1( F15_bl.Mach.data', fdcmv', Maq', 'pchip' )';

fclmax = max(abs(fclvq),[],2);
fcdmax = max(abs(fcdvq),[],2);
fcmmax = max(abs(fcmvq),[],2);
fdclmax = max(abs(fdclvq),[],2);
fdcdmax = max(abs(fdcdvq),[],2);
fdcmmax = max(abs(fdcmvq),[],2);

