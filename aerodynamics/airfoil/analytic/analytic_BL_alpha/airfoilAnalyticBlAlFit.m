function [fcn,fcm] = airfoilAnalyticBlAlFit(alpha_deg,c_N,c_m,varargin)
% airfoilAnalyticBlAlFit fits the analytic normal force coefficient model
%   from Beddoes-Leishman [1].
%   The functions returns the parameters of the analytic function.
% 
% Inputs:
%   alpha_deg           angle of attack data points (1xN array), in deg
%   c_N                 normal force coefficient data points (1xN array),
%                       in deg
%   vis_flag            flag if analytic fit should be visualized (scalar
%                       bolean)
% 
% Outputs:
%   fcn             parameters of the analytic lift coefficient function
%                       fcn(1): lift curve slope (c_N_alpha in [1], eq. (20)), in 1/deg
%                      	fcn(2): zero lift angle of attack (alpha_0), in deg
%                     	fcn(3): alpha_1 in [1], eq. (21)
%                      	fcn(4): S_1 in [1], eq. (21)
%                      	fcn(5): S_2 in [1], eq. (21)
%                      	fcn(6): efficiency factor in [1], eq. (23)
%   fcm             parameters of the analytic pitching moment coefficient
%                   function
%                       fcm(1): pitching moment coefficient at zero lift (c_m0)
%                       fcm(2): K0 in [1], eq. (22)
%                       fcm(3): K1 in [1], eq. (22)
%                       fcm(4): K2 in [1], eq. (22)
%                       fcm(5): m in [1], eq. (22)
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdf/10.2514/6.1989-1319
% 

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

alpha_max = 25;
alpha_min = -5;

% lift coefficient function
liftCurve = @airfoilAnalyticBlAlCn;

% pitching moment function
momentCurve = @airfoilAnalyticBlCm;

% lift coefficient options
optsCn.Lower = [ 0.03 -7 5 0.75 0.75 0.8 ]';
optsCn.StartPoint = [ 0.11 0 15 1.5 1.5 0.95 ]';
optsCn.Upper = [ 0.3 7 25 20 20 1 ]';

options = optimoptions('lsqcurvefit');
options.MaxFunctionEvaluations = 50000;

% fit lift coefficient
% x = alpha_deg;
idx = alpha_deg>=-4 & alpha_deg <= 4;
x = [alpha_deg,repmat(alpha_deg(idx),1,100)];
c_Nx = [c_N,repmat(c_N(idx),1,100)];
fcn = lsqcurvefit( liftCurve, optsCn.StartPoint, x, c_Nx, optsCn.Lower, optsCn.Upper, options );

% pitching moment coefficient options
optsCm.Lower = [ -0.3 -0.2 -0.6 0 0.5 ]';
optsCm.StartPoint = [ 0 0 0 0.03 2 ]';
optsCm.Upper = [ 0.3 0.2 0.1 0.7 5 ]';

% options = optimoptions('lsqcurvefit','OptimalityTolerance',1e-16,'FunctionTolerance',1e-16,'Display','iter-detailed');
options = optimoptions('lsqcurvefit');
options.MaxFunctionEvaluations = 50000;

% fit pitching moment coefficient
[c_L_alpha_max,alpha_0] = airfoilAnalyticBlClAlphaMax( fcn );
f_st = airfoilDynStallFst(c_N,c_L_alpha_max,alpha_deg-alpha_0);
fcm = lsqcurvefit(  @(xxx,data) momentCurve(xxx,data,c_N), optsCm.StartPoint, f_st, c_m, optsCm.Lower, optsCm.Upper, options );


if visualize
    % plot result
    alpha_eval = -5:0.1:30;

    figure
    subplot(2,2,1)
    plot(alpha_deg,c_N,'x')
    hold on
    c_L_eval = airfoilAnalyticBlAlCn(fcn,alpha_eval);
    plot(alpha_eval,c_L_eval)
    grid on
    xlim([alpha_min,alpha_max])
    xlabel('angle of attack, deg')
    ylabel('lift coefficient, -')
    legend('data','analytic function','location','southeast')
    
    subplot(2,2,3)
    plot(alpha_deg,c_m,'x')
    hold on
    [c_L_alpha,alpha_0] = airfoilAnalyticBlClAlphaMax(fcn);
    f_eval = airfoilDynStallFst( c_L_eval, c_L_alpha, alpha_eval-alpha_0 );
    c_m_eval = airfoilAnalyticBlCm(fcm,f_eval,c_L_eval);
    plot(alpha_eval,c_m_eval)
    grid on
    xlim([alpha_min,alpha_max])
    xlabel('angle of attack, deg')
    ylabel('pitching moment coefficient, -')
    legend('data','analytic function','location','northwest')
    
    subplot(2,2,4)
    f = airfoilDynStallFst( c_N, c_L_alpha, alpha_deg-alpha_0 );
    plot(alpha_deg,f,'x')
    hold on
    plot(alpha_eval,f_eval)
    grid on
    xlim([alpha_min,alpha_max])
    ylim([0 1])
    xlabel('angle of attack, deg')
    ylabel('separation point location, -')
    legend('data','analytic function','location','northeast')
    
end

end