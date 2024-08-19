function [fcl,fcd] = airfoilAnalytic9090AlFit(alpha_deg,c_l,c_d,varargin)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~isempty(varargin)
    visualize = varargin{1};
else
    visualize = false;
end

len_cl = length(c_l);

% reshape inputs
alpha_deg = alpha_deg(:)';
alpha_deg_cd = alpha_deg(:)';


% add data if not enough data points are given
num_data_min_cd = 10;
if size(c_d,2) < num_data_min_cd
    alpha_deg_cd_add = (min(alpha_deg_cd):((max(alpha_deg_cd)-min(alpha_deg_cd))/(num_data_min_cd-1)):max(alpha_deg_cd));
    c_d = interp1( alpha_deg_cd, c_d, alpha_deg_cd_add, 'pchip' );
    alpha_deg_cd = alpha_deg_cd_add;
end
num_data_min_cl = 9;
if size(c_l,2) < num_data_min_cl
    alpha_deg_cl_add = (min(alpha_deg):((max(alpha_deg)-min(alpha_deg))/(num_data_min_cl-1)):max(alpha_deg));
    c_l = interp1( alpha_deg, c_l, alpha_deg_cl_add, 'pchip' );
    alpha_deg = alpha_deg_cl_add;
end


% add help data if there is no data for high angles of attack
alpha_min = min(alpha_deg);
alpha_max = max(alpha_deg);
alpha_min_max = mean( [abs(alpha_min),alpha_max] );
if max(alpha_deg) < 30
    alpha_deg(end+1:end+len_cl*2) = repmat([45,90],1,len_cl);
    alpha_deg_cd(end+1:end+2) = [45,90];
    c_l(end+1:end+len_cl*2) = repmat([1,0],1,len_cl);
    c_d(end+1:end+2) = [1.1,1.8];
    alpha_cd_add = [ mean([alpha_max*[1,1],45]) ];
    c_d_add = interp1(alpha_deg_cd,c_d,alpha_cd_add,'pchip');
    alpha_deg_cd(end+1:end+length(alpha_cd_add)) = alpha_cd_add;
    c_d(end+1:end+length(alpha_cd_add)) = c_d_add;
end
if min(alpha_deg) > -30
    alpha_deg = [-90,-45,alpha_deg];
    alpha_deg_cd = [-90,-45,alpha_deg_cd];
    c_l = [0,-1,c_l];
    c_d = [1.8,1.1,c_d];
    alpha_cd_add = mean([alpha_min*[1,1],-45]);
    c_d_add = interp1(alpha_deg_cd(1:len_cl),c_d(1:len_cl),alpha_cd_add,'pchip');
    alpha_deg_cd = [alpha_deg_cd, alpha_cd_add ];
    c_d = [ c_d, c_d_add ];
end


% add c_L = 0 for alpha = 90deg (many times, so that it is strongly
% considered)
len = length(alpha_deg);
lenE = 3*len;
alpha_deg_ext = [ alpha_deg, -90*ones(1,lenE), 90*ones(1,lenE) ];
c_l_ext = [ c_l, zeros(1,lenE), zeros(1,lenE) ];


% lift coefficient function
liftCurve = @airfoilAnalytic9090AlCl;

% use normalized drag coefficient function for curve fitting in order to
% minimize the relative error instead of the absolute error (much more
% accurate for small drag coefficients)
dragCurve = @airfoilAnalytic9090AlCdNorm;



% lift coefficient options
% optsCl = fitoptions( 'Method', 'NonlinearLeastSquares' );
% optsCl.Display = 'Off';
% optsCl.Exclude = abs(alpha_deg_ext)>90;
optsCl.Lower = [ -7 0.03 0.2 2 2 0.2 0.2 0.2 0.2 -5 -5 ]';
optsCl.StartPoint = [ 0 0.1 1 10 10 0.5 0.5 0.5 0.5 0 0 ]';
optsCl.Upper = [ 7 0.3 3 30 30 2 2 2 2 5 5 ]';

options = optimoptions('lsqcurvefit');
options.MaxFunctionEvaluations = 50000;

% fit lift coefficient
x = alpha_deg_ext;
fcl = lsqcurvefit( liftCurve, optsCl.StartPoint, x, c_l_ext, optsCl.Lower, optsCl.Upper, options );



eFacMin = 0.1;
eFacMid = 1;
eFacMax = 5;

filterMin = 1;
filterMid = 7;
filterMax = max( filterMid, 1*alpha_min_max );
shiftDiffMax = 20;

% drag coefficient options
% optsCd = fitoptions( 'Method', 'NonlinearLeastSquares' );
% optsCd.Display = 'Off';
optsCd.Lower = [ 0 0 -10 filterMin -shiftDiffMax eFacMin eFacMin -1.5 0.75 eFacMin -shiftDiffMax 0 ]';
optsCd.StartPoint = [ 0.1 1e-1 0 filterMid 0 eFacMid eFacMid -0.6 0.9 eFacMid 0 0.1 ]';
optsCd.Upper = [ 0.2 1 10 filterMax shiftDiffMax eFacMax eFacMax 0 1.0 eFacMax shiftDiffMax 0.9 ]';

% fit drag coefficient
x = alpha_deg_cd;
fcd = lsqcurvefit( @(xxx,data) dragCurve(xxx,data,alpha_deg_cd,c_d), optsCd.StartPoint, x, ones( size(c_d) ), optsCd.Lower, optsCd.Upper, options );



if visualize
    % plot result
    alpha_eval = -90:0.1:90;

    figure
    subplot(2,1,1)
    plot(alpha_deg_ext,c_l_ext,'x')
    hold on
    plot(alpha_eval,airfoilAnalytic9090AlCl(fcl,alpha_eval))
    grid on
    xlim([-90,90])
    xlabel('alpha, deg')
    ylabel('c_L, -')
    legend('data','analytic function','location','southeast')

    subplot(2,1,2)
    plot(alpha_deg_cd,c_d,'x')
    hold on
    plot( alpha_eval, airfoilAnalytic9090AlCd(fcd,alpha_eval) )
    grid on
    xlim([-90,90])
    xlabel('alpha, deg')
    ylabel('c_D, -')
    legend('data','analytic function','location','southeast')
    
end

end