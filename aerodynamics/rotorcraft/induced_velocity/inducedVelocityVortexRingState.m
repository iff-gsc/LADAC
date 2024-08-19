function [lambda_i] = inducedVelocityVortexRingState(Vx_in, Vz_in) %#codegen
% inducedVelocityVortexRingState computes the induced velocity of a rotor.
%
%   The computation is based on the momentum theory. The equation is
%   solved according to [1, page 153]. The computation is non conducted
%   with the dimensionless lambda but with absolute velocities.
%   This function eliminates the singularity of momentum theory in
%   vortex ring state. This Vortex Ring State model uses third-order
%   polynomials, that connects stepwise the two branches of climb and
%   fast decent. [2, page 14]. This model is based on real rotor test data.
%   This model creates an instability for the vortex ring state. Within
%   the defined boundary the real part of the eigenvalue of heave damping
%   become positive.
%   
% Literature:
% [1]   Van der Wall, B. G. (2015): Grundlagen der Hubschrauber-
%       Aerodynamik. Springer.
% 
% [2]   Johnson, Wayne (2005): Model for Vortex Ring State Influence on
%       Rotorcraft Flight Dynamics. NASA Ames Research Center.
%
% Syntax: [v_i] = inducedVelocityVortexRingState(V_x, V_z) 
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

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% The factor kappa is introduced to account for additional induced losses
% for further information see [2, p.14] (kappa > 0)
kappa = 1.0;

% The Parameter f allows the instability in VRS to be reduced or suppressed
% for further information see [2, p.14] (f > 0)
f     = 1.0;

% Scale V_x and V_z by v_h
Vx = abs(Vx_in);
Vz = Vz_in;

%% Parameters for Vortex Ring State Model according to [2, p.24]
VzD   = -0.20;
VzN   = -0.45;
V_v_N = +0.85;
VzX   = -1.50;
V_v_X = +1.25;
VzE   = -2.00;
VxM   = +0.95;

%% Vortex Ring State Model according to [2, p.23]

% baseline model is used
delta_Vz_v = 0;

if (Vz < 0) && (Vx < VxM)
    
    % stability boundary
    VzDID = VzD;
    VzNID = 0.5*(VzN + VzX ) + 0.5*(VzN - VzX )* (1 - (Vx /VxM )^2 )^0.2;
    VzXID = 0.5*(VzN + VzX ) - 0.5*(VzN - VzX )* (1 - (Vx /VxM )^2 )^1.5;
    VzEID = VzE + (VzXID - VzX );
    
    if (VzEID < Vz) && (Vz < VzDID)
        
        % use the slope of baseline function ( slope of MT is possible too)
        dv_base_dVz = @(V_eval) (inducedVelocityBaselineModel(Vx, V_eval+0.01) - inducedVelocityBaselineModel(Vx, V_eval) ) / 0.01;
        v_mom       = @(V_eval) inducedVelocityBaselineModel(Vx, V_eval) ;
        
        % identify Δ(V z +v) = a + bV z + cV z2 + dV z3 ;
        % otherwise momentum theory is used
                
        if (VzNID <= Vz) && (Vz < VzDID)
            
            % match at VzDID : Δ (V z +v) = d Δ (V z +v)/dV z = 0
            d_Vz_v_D     = 0;
            d_Vz_v_dVz_D = 0;
            
            % match at VzNID : Δ (V z +v) = ((V z +v) N –(V zN +v Nmom )) (1–(V x /V xM ) 6 ) 0.5 and d Δ (V z +v)/dV z = –(1 + dv base /dV z )
            d_Vz_v_N     = ( V_v_N -( VzN + v_mom(VzNID) )) * ( 1 - ( Vx / VxM ) )^0.5;
            d_Vz_v_dVz_N = -(1 + dv_base_dVz(VzNID) );
            
            % solve linear system to find coeffs for v = b*Vz + c*Vz^2 + d*Vz^3;
            A_mat = [1 VzDID   VzDID^2      VzDID^3;    % point D
                     1 VzNID   VzNID^2      VzNID^3;    % point N
                     0     1   2*VzDID      3*VzDID^2;  % slope at point D
                     0     1   2*VzNID      3*VzNID^2]; % slope at point N
                 
        
            rhs = [d_Vz_v_D; d_Vz_v_N; d_Vz_v_dVz_D; d_Vz_v_dVz_N];
        
            linear_coffs = A_mat \ rhs;
            
            a = linear_coffs(1);
            b = linear_coffs(2);
            c = linear_coffs(3);
            d = linear_coffs(4);
            
            delta_Vz_v = a + b * Vz + c * Vz^2 + d*Vz^3 ;
                   
        end
        
        if (VzXID <= Vz) && (Vz < VzNID)
            % match at VzNID : Δ (V z +v) = ((V z +v) N –(V zN +v Nmom )) (1–(V x /V xM ) 6 ) 0.5 and d Δ (V z +v)/dV z = –(1 + dv base /dV z )
            d_Vz_v_N     = ( V_v_N -( VzN + v_mom(VzNID) )) * ( 1 - ( Vx / VxM ) )^0.5;
            d_Vz_v_dVz_N = -(1 + dv_base_dVz(VzNID) );
            
            
            % match at VzXID : Δ (V z +v) = ((V z +v) X –(V zX +v Xmom )) (1–(V x /V xM ) 6 ) 0.5 and d Δ (V z +v)/dV z = –(1 + dv base /dV z )
            d_Vz_v_X     = ( V_v_X -( VzX + v_mom(VzXID) )) * ( 1 - ( Vx / VxM ) )^0.5;
            d_Vz_v_dVz_X = -(1 + dv_base_dVz(VzXID));
            
            % solve linear system to find coeffs for v = b*Vz + c*Vz^2 + d*Vz^3;
            A_mat = [1 VzXID   VzXID^2      VzXID^3;    % point X
                     1 VzNID   VzNID^2      VzNID^3;    % point N
                     0     1   2*VzXID      3*VzXID^2;  % slope at point X
                     0     1   2*VzNID      3*VzNID^2]; % slope at point N
                 
        
            rhs = [d_Vz_v_X; d_Vz_v_N; d_Vz_v_dVz_X; d_Vz_v_dVz_N];
        
            linear_coffs = A_mat \ rhs;
            
            a = linear_coffs(1);
            b = linear_coffs(2);
            c = linear_coffs(3);
            d = linear_coffs(4);
            
            delta_Vz_v = a + b * Vz + c * Vz^2 + d*Vz^3 ;
            
        end
        
        if (VzEID < Vz) && (Vz < VzXID)
            % match at VzXID : Δ (V z +v) = ((V z +v) X –(V zX +v Xmom )) (1–(V x /V xM ) 6 ) 0.5 and d Δ (V z +v)/dV z = –(1 + dv base /dV z )
            d_Vz_v_X     = (V_v_X -(VzX + v_mom(VzXID)))*(1-(Vx / VxM ) )^0.5;
            d_Vz_v_dVz_X = -(1 + dv_base_dVz(VzXID) );
            
            % match at VzEID : Δ (V z +v) = 0 (not matching slope, so a=0)
            d_Vz_v_E     = 0;
            
                        % solve linear system to find coeffs for v = b*Vz + c*Vz^2 + d*Vz^3;
            A_mat = [VzXID   VzXID^2      VzXID^3;    % point X
                     VzEID   VzEID^2      VzEID^3;    % point E
                         1   2*VzXID      3*VzXID^2];  % slope at point X
                      
            rhs = [d_Vz_v_X; d_Vz_v_E; d_Vz_v_dVz_X];
        
            linear_coffs = A_mat \ rhs;
            
            a = 0;
            b = linear_coffs(1);
            c = linear_coffs(2);
            d = linear_coffs(3);
            
            delta_Vz_v = a + b * Vz + c * Vz^2 + d*Vz^3 ;

        end
 
    else
        
        % baseline model is used
        delta_Vz_v = 0;
        
    end
else
        % baseline model is used
        delta_Vz_v = 0;
end

% calculate induced velocity of baseline model and add delta_Vz_v
v = kappa * ( inducedVelocityBaselineModel(Vx, Vz) + f * delta_Vz_v);

%% Calculate and scale output variables

% scale induced velocity from baseline model with v_h
lambda_i = v;

end
