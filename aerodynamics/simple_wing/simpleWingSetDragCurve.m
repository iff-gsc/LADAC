function wing = simpleWingSetDragCurve( wing )
% simpleWingSetDragCurve calculates the drag coefficient with respect to
% the angle of attack.
% The inputs of this function are characteristic points on the drag curve.
% In this function these points are interpolated with splines. 
% The outputs of this function is the angle of attack vector X and the 
% drag coefficient vector Y 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% angle of attack breakpoints
x(1) = -90;
x(2) = -wing.polar.params.alpha_kink;
x(3) = -wing.polar.params.alpha_linend;
x(4) = 0;
x(5) = wing.polar.params.alpha_linend;
x(6) = wing.polar.params.alpha_kink;
x(7) = 90;

% drag coefficients corresponding to the angle of attack breakpoints
y(1) = wing.polar.params.C_Dmax;
y(2) = wing.polar.params.C_Dkink;
y(3) = wing.polar.params.C_D0 +(wing.polar.params.C_L0+wing.polar.params.C_Lalpha*x(3)*pi/180)^2*wing.polar.params.k;     
y(4) = wing.polar.params.C_D0 + wing.polar.params.C_L0^2*wing.polar.params.k;
y(5) = wing.polar.params.C_D0 +(wing.polar.params.C_L0+wing.polar.params.C_Lalpha*x(5)*pi/180)^2*wing.polar.params.k;        
y(6) = wing.polar.params.C_Dkink;
y(7) = wing.polar.params.C_Dmax;

% derivative of the drag coefficient corresponding to the angle of attack
% breakpoints
dy(1) = 0;
dy(2) = -1.4*(wing.polar.params.C_Dmax-wing.polar.params.C_Dkink)/(90-wing.polar.params.alpha_kink);
dy(3) = pi/180*2*(wing.polar.params.C_L0+wing.polar.params.C_Lalpha*x(3)*pi/180)*wing.polar.params.k*wing.polar.params.C_Lalpha;
dy(4) = pi/180*2*wing.polar.params.C_L0*wing.polar.params.C_Lalpha*wing.polar.params.k;
dy(5) = pi/180*2*(wing.polar.params.C_L0+wing.polar.params.C_Lalpha*x(5)*pi/180)*wing.polar.params.k*wing.polar.params.C_Lalpha;
dy(6) = 1.4*(wing.polar.params.C_Dmax-wing.polar.params.C_Dkink)/(90-wing.polar.params.alpha_kink);
dy(7) = 0;

% initialization
M = zeros(2,4);
N = zeros(2,4);

% distance between the points of the interpolated data (-90:Da:90)
Da = 1;

x(x==0) = 0.0001;   %avoid division by zero

% for every interval between the breakpoints a spline (a3*x^3+a2+x^2+a1+a0)
% -> 4 coefficients (a3,a2,a1,a0) to determine -> system of 4 equations ->
% matrix equation with 4 rows
for u = 1:length(x)-1
    for i =1:4
        M(:,i) = x(u:u+1).^(i-1);           % the points of the spline
        N(:,i) = (i-1)*x(u:u+1).^(i-2);     % the derivative of the spline
    end

    A = [M;N];                              

%     a = inv(A) * [y(u:u+1),dy(u:u+1)]';     % determine coefficient vector (a0, a1, a2, a3)
    a = A \ [y(u:u+1),dy(u:u+1)]';     % determine coefficient vector (a0, a1, a2, a3)
    
    X{u} = x(u):Da:x(u+1);                  % x axis vector (angle of attack)
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
Y(diff(X) < 0.5*Da) = [];
X(diff(X) < 0.5*Da) = [];

wing.polar.alpha = deg2rad(X);
wing.polar.C_D = Y;

end
