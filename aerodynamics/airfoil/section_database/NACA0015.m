function NACA0015 = NACA0015(varargin)
%% NACA0015 airfoil data for Re = 160,000
% from [1], p.31/32
% 
%   [1] Sheldahl, R. E, Klimas, P. C. (1981). Aerodynamic Characteristics
%       of Seven Symmetrical Airfoil Sections Through 180-Degree Angle of
%       Attack for Use in Aerodynamic Analysis of Vertical Axis Wind
%       Turbines. SAND80-2114. Sandia National Laboratories.
%       https://www.osti.gov/servlets/purl/6548367

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~isempty(varargin)
    visualize = varargin{1};
else
    visualize = false;
end

alpha_1 = [0:27]';
cl_1 = [.0000 .1100 .2200 .3300 .4400 .5500 .6299 .7150 .7851 .8311 .8322 .7632 .5936 .3548 .2371 .2376 .2665 .3098 .3567 .4066 .4575 .5087 .5611 .6148 .6685 .7224 .7771 .8382]';
cd_1 = [.0115 .0117 .0120 .0124 .0132 .0142 .0160 .0176 .0193 .0212 .0233 .0256 .0281 .0302 .1040 .1770 .1970 .2170 .2380 .2600 .2820 .3050 .3290 .3540 .3790 .4050 .4320 .4600]';


alpha_2 = [30:5:180]';
cl_2 = [
.8550
.9800
1.0350
1.0500
1.0200
.9550
.8750
.7600
.6300
.5000
.3650
.2300
.0900
-.0500
-.1850
-.3200
-.4500
-.5750
-.6700
-.7600
-.8500
-.9300
-.9800
-.9000
-.7700
-.6700
-.6350
-.6800
-.8500
-.6600
-.0000];

cd_2 = [
.5700
.7450
.9200
1.0750
1.2150
1.3450
1.4700
1.5750
1.6650
1.7350
1.7800
1.8000
1.8000
1.7800
1.7500
1.7000
1.6350
1.5550
1.4650
1.3509
1.2250
1.0850
.9250
.7550
.5750
.4200
.3200
.2300
.1400
.0550
.0250
];

alpha = [alpha_1; alpha_2];
cl = [cl_1; cl_2];
cd = [cd_1; cd_2];

% symmetric airfoil
NACA0015.alpha = [-flipud(alpha); alpha(2:end)];
NACA0015.cl = [-flipud(cl); cl(2:end)];
NACA0015.cd = [flipud(cd); cd(2:end)];

if visualize

    alpha_q = linspace(-180,180,500);
    cl_q = interp1(NACA0015.alpha, NACA0015.cl, alpha_q, 'PCHIP', 0);
    cd_q = interp1(NACA0015.alpha, NACA0015.cd, alpha_q, 'PCHIP', 0);

    figure(17)
    plot(alpha_q, [cl_q; cd_q])
    hold on
    plot(NACA0015.alpha,[NACA0015.cl,NACA0015.cd],'x')
    grid on
    hold off
    
end

end