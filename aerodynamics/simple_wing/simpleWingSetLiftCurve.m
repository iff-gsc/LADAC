function wing = simpleWingSetLiftCurve( wing )
% simpleWingSetLiftCurve calculates the lift coefficient with respect to
% the angle of attack.
% The inputs of this function are characteristic points on the lift curve.
% In this function these points are interpolated with splines. 
% The outputs of this function is the angle of attack vector X and the 
% lift coefficient vector Y  

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% angle of attack breakpoints
x(1) = -90;
x(2) = -wing.polar.params.alpha_max2;
x(3) = -wing.polar.params.alpha_min;
x(4) = -wing.polar.params.alpha_max;
x(5) = -wing.polar.params.alpha_linend;
x(6) = wing.polar.params.alpha_linend;
x(7) = wing.polar.params.alpha_max;
x(8) = wing.polar.params.alpha_min;
x(9) = wing.polar.params.alpha_max2;
x(10) = 90;

% drag coefficients corresponding to the angle of attack breakpoints
y(1) = 0;
y(2) = -wing.polar.params.C_Lmax2;
y(3) = -wing.polar.params.C_Lmin+wing.polar.params.C_L0;
y(4) = -wing.polar.params.C_Lmax+2*wing.polar.params.C_L0;
y(5) = wing.polar.params.C_L0+wing.polar.params.C_Lalpha*x(5)*pi/180;
y(6) = wing.polar.params.C_L0+wing.polar.params.C_Lalpha*x(6)*pi/180;
y(7) = wing.polar.params.C_Lmax;
y(8) = wing.polar.params.C_Lmin;
y(9) = wing.polar.params.C_Lmax2;
y(10) = 0;

% derivative of the drag coefficient corresponding to the angle of attack
% breakpoints
dy(1) = -0.5*pi/180;
dy(1) = -wing.polar.params.C_Lmax2 * pi / ( pi - wing.polar.params.alpha_max2*pi/180 )*pi/180;
dy(2) = 0;
dy(3) = 0;
dy(4) = 0;
dy(5) = wing.polar.params.C_Lalpha*pi/180;
dy(6) = wing.polar.params.C_Lalpha*pi/180;
dy(7) = 0;
dy(8) = 0;
dy(9) = 0;
dy(10) = -0.5*pi/180;
dy(10) = -wing.polar.params.C_Lmax2 * pi / ( pi - wing.polar.params.alpha_max2*pi/180 )*pi/180;


% initialization
M = zeros(2,4);
N = zeros(2,4);

% distance between the points of the interpolated data (-90:Da:90)
Da = 1;

i = 0;

% for every interval between the breakpoints a spline (a3*x^3+a2+x^2+a1+a0)
% -> 4 coefficients (a3,a2,a1,a0) to determine -> system of 4 equations ->
% matrix equation with 4 rows
for u = 1:length(x)-1
    for i =1:4
        M(:,i) = x(u:u+1).^(i-1);           % the points of the spline
        N(:,i) = (i-1)*x(u:u+1).^(i-2);     % the derivative of the spline
    end

    A = [M;N];

    a = inv(A) * [y(u:u+1),dy(u:u+1)]';     % determine coefficient vector (a0, a1, a2, a3)

    
    X{u} = x(u):Da:x(u+1);               % x axis vector (angle of attack)
    B = zeros(length(X{u}),length(a));
    for j = 1:length(a)
        B(:,j) = X{u}.^(j-1);
    end
    for k = 1:length([X{u}])
        Y{u}(k) = B(k,:) * a;               % y axis vector (drag coefficient)
    end
    
end

X = [X{:}];
Y = [Y{:}];

% assure that the x axis vector is monotonically increasing (for lookup
% table)
Y(diff(X) == 0) = [];
X(diff(X) == 0) = [];

wing.polar.alpha = deg2rad(X);
wing.polar.C_L = Y;

end
