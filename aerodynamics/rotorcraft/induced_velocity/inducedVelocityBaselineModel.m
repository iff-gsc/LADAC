function [lambda_i] = inducedVelocityBaselineModel(Vx, Vz) %#codegen
% inducedVelocityBaselineModel computes the induced velocity of a rotor.
%
%   The computation is based on the momentum theory. The equation is
%   solved according to [1, page 153]. The computation is non conducted
%   with the dimensionless lambda but with absolute velocities.
%   This function eliminates the singularity of momentum theory in
%   vortex ring state. This baseline model uses a a third-order polynomial,
%   that connects the two branches of climb and fast decent. [2, page 14].
%
% Literature:
% [1]   Van der Wall, B. G. (2015): Grundlagen der Hubschrauber-
%       Aerodynamik. Springer.
% 
% [2]   Johnson, Wayne (2005): Model for Vortex Ring State Influence on
%       Rotorcraft Flight Dynamics. NASA Ames Research Center.
%
% Syntax: [lambda_i] = inducedVelocityBaselineModel(V_x, V_z, v_h) 
%
% Inputs:
%   V_x         forward velocity of the rotor plane relative to the air
%               (scalar), in m/s
%   V_z         velocity of the propeller relative to the air perpendicular
%               to the rotor. Positive V_z means rotor is ascending,
%               negative V_z means that the rotor is descending.
%               (scalar), in m/s                             
%
% Outputs:
%   v_i         induced velocity of the propeller (scalar), in m/s
%
% See also: InducedVelocity
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


%% Parameters for baseline model according to [2, p.24]
VzA = -1.50;
VzB = -2.10;
VxC = +0.75;

%% Baseline Model according to [2, p.23]

if (Vz >= 0) || (Vx >= VxC)
    
    % momentum theory is used
    v = inducedVelocityMomentumTheory(Vx, Vz) ;
        
else   
      
    VzAID = VzA + 0.2 * (Vx / VxC )^2;
    VzBID = VzB + 0.2 * (Vx / VxC )^2;
    
    if (Vx / VxC > 0.5)
        VzBID = VzBID + 0.7 *(VzAID - VzBID ) * (2*Vx / VxC - 1)^3;
    end
    
    if(VzBID <= Vz) && (Vz <= VzAID)   % baseline model is used
        
        % use momentum theory to calculate induced velocity in A
        [v_AID] = inducedVelocityMomentumTheory(Vx, VzAID) ;
        
        % use finite difference to calculate derivate dv / dVZ in A
        step_size = 0.01;   
        
        [v_AID_delta] = inducedVelocityMomentumTheory(Vx, VzAID + step_size) ;
        dv_dVz_A = (v_AID_delta - v_AID) / step_size;
              
        % use momentum theory to calculate induced velocity in B
        v_BID = inducedVelocityMomentumTheory(Vx, VzBID) ;
        
        % solve linear system to find coeffs for v = b*Vz + c*Vz^2 + d*Vz^3;
        A_mat = [VzAID  VzAID^2     VzAID^3;
                 VzBID  VzBID^2     VzBID^3;
                 1      2*VzAID   3*VzAID^2];
        
        rhs = [v_AID; v_BID; dv_dVz_A];
        
        linear_coffs = A_mat \ rhs;
        
        b = linear_coffs(1);
        c = linear_coffs(2);
        d = linear_coffs(3);
        
        % calculate induced normalized velocity
        v = b*Vz + c*Vz^2 + d*Vz^3;       
       
    else
        
        % momentum theory is used
        v = inducedVelocityMomentumTheory(Vx, Vz) ;      
       
    end
    
end

%% Calculate and scale output variables

% calculate induced velocity
lambda_i = v;


end
