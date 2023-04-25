function [fcl,fcd,fcm] = airfoilAnalytic0515AlFit(alpha_deg,Ma,c_L,c_D,c_m,varargin)
% airfoilAnalytic0515AlFit performs an analytic fit of airfoil coefficients
% data for different angles of attack and Mach numbers.
%   For the analytic fit, the analytic function airfoilAnalytic0515De is
%   used.
% 
% Inputs:
%   alpha_deg      	angle of attack (1xN array), in deg
%   Ma             	Mach number (1xN array), dimensionless
%   c_L            	lift coefficient (1xN array), dimensionless
%   c_D           	drag coefficient (1xN array), dimensionless
%   c_m            	pitching moment coefficient (1xN array), dimensionless
%   vis_flag       	flag if analytic fit should be visualized (scalar
%                  	bolean)
% 
% Outputs:
%   fcl             solution for the analtic function coefficients for
%                 	the lift coefficient (6x1 array or 6xN array):
%                       fcl(1,:):  	alpha_0
%                       fcl(2,:):  	lift curve slope for Ma=0 and c_L=0
%                       fcl(3,:):  	magnitude of stall lift coefficient (usually 
%                                   negative)
%                       fcl(4,:):  	angle of attack where the mean stall occurs
%                       fcl(5,:):  	additional correction factor for Prandtl-
%                                   Glauert factor (usually close to 1)
%                       fcl(6,:):  	abruptness factor of the stall onset
%   fcd            	solution for the analtic function coefficients for
%                 	the drag coefficient (6x1 array or 6xN array):
%                       fcd(1,:): 	minimum drag coefficient
%                       fcd(2,:): 	multiplicator for parabola
%                       fcd(3,:): 	angle of attack, where the minimum drag 
%                                   coefficient occurs
%                       fcd(4,:):  	steepness of the drag increase due to stall
%                       fcd(5,:): 	angle of attack where the attached flow and
%                                   detached flow
%                                   models are blended
%                       fcd(6,:): 	maximum drag coefficient at 90 degrees angle of
%                                   attack (does not need to be accurate)
%   fcm            	solution for the analtic function coefficients for
%                  	the pitching moment coefficient (5x1 array or 5xN
%                  	array):
%                     	fcm(1,:): 	minimum pitching moment coefficient in attached flow region
%                       fcm(2,:):	multiplicator for parabola
%                       fcm(3,:):	angle of attack, where the minimum pitching moment
%                                   coefficient in attached flow region occurs
%                       fcm(4,:): 	steepness of the drag increase due to stall
%                       fcm(5,:): 	angle of attack where the attached flow and detached flow
%                                   models are blended
%                       fcm(6,:): 	maximum pitching moment coefficient at 45 degrees angle of
%                                  	attack (does not need to be accurate)
% 
% See also: lsqcurvefit, airfoilAnalytic0515AlCl, airfoilAnalytic0515AlCd

% Disclamer:
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

% assure row vectors
alpha_deg = alpha_deg(:)';
c_L = c_L(:)';
c_D = c_D(:)';
c_m = c_m(:)';


% add help data if there is no data for low angles of attack
alpha_min = min(alpha_deg);
if alpha_min > -1
    alpha_deg_cl_ext1 = [ -2.5, alpha_deg ];
    c_l_ext1 = [ 0, c_L ];
    c_l_ext1(1) = interp1( alpha_deg, c_L, alpha_deg_cl_ext1(1), 'linear', 'extrap' );
    
    n_dupl = 300;
    alpha_deg_cl_ext2 = [ repmat(alpha_deg_cl_ext1(2:4),1,n_dupl), alpha_deg_cl_ext1 ];
    c_l_ext2 = [ zeros(1,3*n_dupl), c_l_ext1 ];
    c_l_ext2(1:3*n_dupl) = interp1( alpha_deg_cl_ext1, c_l_ext1, alpha_deg_cl_ext2(1:3*n_dupl), 'makima' );
    
    n_dupl = 30;
    alpha_deg_cl_ext3 = [ repmat(alpha_deg_cl_ext1([1,5]),1,n_dupl), alpha_deg_cl_ext1 ];
    c_l_ext3 = [ zeros(1,2*n_dupl), c_l_ext1 ];
    c_l_ext3(1:2*n_dupl) = interp1( alpha_deg_cl_ext1, c_l_ext1, alpha_deg_cl_ext3(1:2*n_dupl), 'makima' );
    
    c_l_ext = [ c_l_ext2, c_l_ext3 ];
    alpha_deg_cl_ext = [ alpha_deg_cl_ext2, alpha_deg_cl_ext3 ];
end




% add data if not enough data points are given
num_data_min_cd = 6;
if length(alpha_deg) < num_data_min_cd
    alpha_deg_cd_add = (min(alpha_deg):((max(alpha_deg)-min(alpha_deg))/(num_data_min_cd-1)):max(alpha_deg));
    c_D = interp1( alpha_deg, c_D, alpha_deg_cd_add, 'pchip' );
    alpha_deg_cd = alpha_deg_cd_add;
else
    alpha_deg_cd = alpha_deg;
end
num_data_min_cm = 12;
if length(alpha_deg) < num_data_min_cd
    alpha_deg_cm_add = (min(alpha_deg):((max(alpha_deg)-min(alpha_deg))/(num_data_min_cm-1)):max(alpha_deg));
    c_m = interp1( alpha_deg, c_m, alpha_deg_cm_add, 'pchip' );
    c_l_cm = interp1( alpha_deg, c_L, alpha_deg_cm_add, 'pchip' );
    alpha_deg_cm = alpha_deg_cm_add;
else
    alpha_deg_cm = alpha_deg;
    c_l_cm = c_L;
end
num_data_min_cl = 6;
if length(alpha_deg_cl_ext) < num_data_min_cl
    alpha_deg_cl_add = (min(alpha_deg):((max(alpha_deg)-min(alpha_deg))/(num_data_min_cl-1)):max(alpha_deg));
    c_l_ext = interp1( alpha_deg_cl_ext, c_L, alpha_deg_cl_add, 'pchip');
    alpha_deg_cl_ext = alpha_deg_cl_add;
end




% lift coefficient function
liftCurve = @airfoilAnalytic0515AlCl;

% use normalized drag coefficient function for curve fitting in order to
% minimize the relative error instead of the absolute error (much more
% accurate for small drag coefficients)
dragCurve = @airfoilAnalytic0515AlCdNorm;

momentCurve = @airfoilAnalyticBlCm;


% lift coefficient options
optsCl.Lower = [ -7 0.03 -1.5 4 0.66 0.2 ]';
optsCl.StartPoint = [ -2 0.11 0 7 1 5 ]';
optsCl.Upper = [ 7 0.3 0 30 1.5 20 ]';

options = optimoptions('lsqcurvefit');
options.MaxFunctionEvaluations = 50000;

% fit lift coefficient
xy = grid2Coordinates( alpha_deg_cl_ext, Ma )';
fcl = lsqcurvefit( liftCurve, optsCl.StartPoint, xy, c_l_ext, optsCl.Lower, optsCl.Upper, options );



% add help data if there is no data for low angles of attack
alpha_min = min(alpha_deg_cd);
[~,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, Ma );
if alpha_min > alpha_0
    alpha_deg_cd_ext = [ alpha_0 - (alpha_min-alpha_0), alpha_deg_cd ];
    c_d_ext = [ c_D(1), c_D ];
else
    c_d_ext = c_D;
    alpha_deg_cd_ext = alpha_deg_cd;
end


% drag coefficient options
optsCd.Lower = [ 1e-5 1e-5 -5 0.1 3 0.5 ]';
optsCd.StartPoint = [ 0.001 0.0001 -1 1 10 1.5 ]';
optsCd.Upper = [ 0.1 0.0005 -0.1 10 25 3 ]';

% fit drag coefficient
x = alpha_deg_cd_ext(:)';
fcd = lsqcurvefit( @(xxx,data) dragCurve(xxx,data,alpha_deg_cd_ext,c_d_ext), optsCd.StartPoint, x, ones( size(c_d_ext) ), optsCd.Lower, optsCd.Upper, options );


% pitching moment coefficient options
optsCm.Lower = [ -0.3 -0.2 -0.6 0 0.2 ]';
optsCm.StartPoint = [ 0 0 0 0.03 1 ]';
optsCm.Upper = [ 0.3 0.2 0.2*0 0.7 5 ]';

% fit pitching moment coefficient
[c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, Ma );
f_st = airfoilDynStallFst(c_l_cm,c_L_alpha_max,alpha_deg_cm-alpha_0);
c_L = airfoilAnalytic0515AlCl( fcl, grid2Coordinates( alpha_deg_cm, Ma )' );
% c_L = c_L_alpha_max * ( alpha_deg_cm - alpha_0 );
% fcm = lsqcurvefit( momentCurve, optsCm.StartPoint, alpha_deg_cm, c_m, optsCm.Lower, optsCm.Upper, options );
fcm = lsqcurvefit(  @(xxx,data) momentCurve(xxx,data,c_L), optsCm.StartPoint, f_st, c_m, optsCm.Lower, optsCm.Upper, options );


if visualize
    % plot result
    alpha_eval = -10:0.1:15;

    figure
    subplot(2,2,1)
    plot(alpha_deg_cl_ext,c_l_ext,'x')
    hold on
    xy = grid2Coordinates(alpha_eval,Ma)';
    c_L_eval = airfoilAnalytic0515AlCl(fcl,xy);
    plot(alpha_eval,c_L_eval)
    grid on
    xlabel('alpha, deg')
    ylabel('c_L, -')
    legend('data','analytic function','location','southeast')
    
    subplot(2,2,2)
    plot(alpha_deg_cd_ext,c_d_ext,'x')
    hold on
    plot(alpha_eval,airfoilAnalytic0515AlCd(fcd,alpha_eval));
    grid on
    ylim([0,inf])
    xlabel('Angle of attack, deg')
    ylabel('Drag coefficient, -')
    legend('data','analytic function','location','best')
    
    subplot(2,2,3)
    plot(alpha_deg_cm,c_m,'x')
    hold on
    f_st_eval = airfoilDynStallFst(c_L_eval,c_L_alpha_max,alpha_eval-alpha_0);
%     c_m_eval = airfoilAnalyticBlCm(fcm,f_st_eval,c_L_eval);
%     c_L_lin = c_L_alpha_max * ( alpha_eval - alpha_0 )';
    c_m_eval = airfoilAnalyticBlCm(fcm,f_st_eval,c_L_eval);
    plot(alpha_eval,c_m_eval);
    grid on
    xlabel('Angle of attack, deg')
    ylabel('Pitching moment coefficient, -')
    legend('data','analytic function','location','best')
    
    subplot(2,2,4)
    plot(alpha_eval,f_st_eval)
    grid on
    xlabel('Angle of attack, deg')
    ylabel('TE separation point, -')

end

end