function [fdcl,fdcd,fdcm] = airfoilAnalytic0515DeFit(alpha_deg,delta,Delta_c_L,Delta_c_D,Delta_c_m,varargin)
% airfoilAnalytic0515DeFit performs an analytic fit of airfoil coefficients
% data for different actuator settings.
%   For the analytic fit, the analytic function airfoilAnalytic0515De is
%   used.
% 
% Inputs:
%   alpha_deg           angle of attack (1xN array), in deg
%   delta               actuator setting (1xN array), in arbitrary unit
%   Delta_c_L           delta lift coefficient due to actuator setting (1xN
%                       array), dimensionless
%   Delta_c_D           delta drag coefficient due to actuator setting (1xN
%                       array), dimensionless
%   Delta_c_m           delta pitching moment coefficient due to actuator 
%                       setting (1xN array), dimensionless
%   vis_flag            flag if analytic fit should be visualized (scalar
%                       bolean)
% 
% Outputs:
%   fdcl                solution for the analtic function coefficients for
%                       the delta lift coefficient (3x1 array)
%   fdcd                solution for the analtic function coefficients for
%                       the delta lift coefficient (3x1 array)
%   fdcm                solution for the analtic function coefficients for
%                       the delta lift coefficient (3x1 array)
% 
% See also: lsqcurvefit, airfoilAnalytic0515De
% 

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
delta = delta(:)';
Delta_c_L = Delta_c_L(:)';
Delta_c_D = Delta_c_D(:)';
Delta_c_m = Delta_c_m(:)';


% lift coefficient function
deltaCoeff = @airfoilAnalytic0515De;


% lift coefficient options
optsDe.Lower = [ -1 1 0.1 ]';
optsDe.StartPoint = [ 0 10 1 ]';
optsDe.Upper = [ 1 20 5 ]';

options = optimoptions('lsqcurvefit');
options.MaxFunctionEvaluations = 50000;

% fit coefficients
xy = grid2Coordinates( alpha_deg, delta )';
fdcl = lsqcurvefit( deltaCoeff, optsDe.StartPoint, xy, Delta_c_L, optsDe.Lower, optsDe.Upper, options );
fdcd = lsqcurvefit( deltaCoeff, optsDe.StartPoint, xy, Delta_c_D, optsDe.Lower, optsDe.Upper, options );
fdcm = lsqcurvefit( deltaCoeff, optsDe.StartPoint, xy, Delta_c_m, optsDe.Lower, optsDe.Upper, options );



if visualize
    % plot result
    alpha_eval = -10:0.1:15;
    delta_eval = delta;

    figure
    subplot(2,2,1)
    plot(alpha_deg,reshape(Delta_c_L,length(alpha_deg),[])','x')
    hold on
    c_L_eval = airfoilAnalytic0515De(fdcl,grid2Coordinates(alpha_eval,delta_eval)');
    plot(alpha_eval,reshape(c_L_eval,length(alpha_eval),[])')
    grid on
    xlabel('alpha, deg')
    ylabel('Delta lift coefficient, -')
    legend('data','analytic function','location','southeast')
    
    subplot(2,2,2)
    plot(alpha_deg,reshape(Delta_c_D,length(alpha_deg),[])','x')
    hold on
    c_D_eval = airfoilAnalytic0515De(fdcd,grid2Coordinates(alpha_eval,delta_eval)');
    plot(alpha_eval,reshape(c_D_eval,length(alpha_eval),[])');
    grid on
    xlabel('Angle of attack, deg')
    ylabel('Delta drag coefficient, -')
    legend('data','analytic function','location','best')
    
    subplot(2,2,3)
    plot(alpha_deg,reshape(Delta_c_m,[],length(alpha_deg)),'x')
    hold on
    c_m_eval = airfoilAnalytic0515De(fdcm,grid2Coordinates(alpha_eval,delta_eval)');
    plot(alpha_eval,reshape(c_m_eval,length(alpha_eval),[])');
    grid on
    xlabel('Angle of attack, deg')
    ylabel('Delta pitching moment coefficient, -')
    legend('data','analytic function','location','best')

end

end