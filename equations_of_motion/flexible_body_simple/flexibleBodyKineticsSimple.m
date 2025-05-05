function [V_Kb_dt,Omega_Kb_dt,eta_from_7_dt2] = ...
    flexibleBodyKineticsSimple( m, I_b, g, structure_red, m_nodes, ...
    cg_nodes, V_Kb, Omega_Kb, M_bg, eta, eta_dt, q, R_b, Q_b )
% flexibleBodyKineticsSimple compute state derivatives flexible equations
%   of motion according to Waszak & Schmidt [1].
%   All outputs are represented in "practical" mean axis system.
%   The model neglects all inertial coupling between the free vibrations
%   model and the rigid body rotational motion.
%   Moreover, the model neglects the effects of variable inertia.
%   Clean models including all inertial coupling and varying inertia can be
%   found in literature, e.g. [2]. However, it is probably difficult to
%   implement them because the highest time derivatives are coupled.
%   A discussion of the mean axis assumptions can be found in [3].
% 
% Inputs:
%   m               total mass (scalar), in kg
%   I_b             total inertia matrix represented in body frame (3x3
%                   array), in kg*m^2
%   g               gravitational acceleration (scalar), in m/s^2
%   structure_red   reduced-orger structure struct (see
%                   structureGetReduced);
%                   the axis system of the structure must be parallel to
%                   the body frame
%   m_nodes         masses of the nodes (1xM array for M nodes), in kg
%   cg_nodes        center of gravity offset of the nodes (3xM array), in m
%   V_Kb            velocity vector (3x1 array) of the rigid body relative
%                   to the earth represented in body-fixed frame, in m/s
%   Omega_Kb        angular velocity vector (3x1 array) of the rigid body
%                   relative to the earth represented in body-fixed frame,
%                   in rad/s
%   M_bg            rotation 3x3 matrix (DCM) from the earth frame (g) to 
%                   the body-fixed frame (b), in 1         
%   eta             Generalized displacement vector excluding the rigid
%                   body motion. The rigid body motion is assumed to be the
%                   first 6 generalized displacements (Nx1 array)
%   eta_dt          Time-derivative of input eta (Nx1 array)
%   q               generalized external force vector excluding gravity
%                   (Nx1 array)
% 
% Outputs:
%   Omega_Kb_dt  	time derivative of the angular velocity vector of the
%                   rigid body relative to the earth represented in 
%                   body-fixed frame (3x1 array), in rad/s^2
%   V_Kb_dt      	time derivative of the velocity vector of the rigid 
%                   body relative to the earth represented in body-fixed
%                   frame (3x1 array), in m/^2
%   eta_from_7_dt2  second time derivative of input eta_from_7
% 
% See also:
%   structureCreate, structureGetReduced, structureGetAcc,
%   rigidBodyKinetics
% 
% Literature:
%   [1] Waszak, M. R. & Schmidt, D. K. (1988). Flight dynamics of
%       aeroelastic vehicles. Journal of Aircraft, 25(6), 563-571.
%   [2] Reschke, C. (2006). Integrated flight loads modelling and analysis
%       for flexible transport aircraft.
%   [3] Schmidt, D. K. (2015). Discussion:"The Lure of the Mean Axes"
%       (Meirovitch, L., and Tuzcu, I., ASME J. Appl. Mech., 74 (3), pp. 
%       497-504). Journal of Applied Mechanics, 82(12).

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2019-2023 Yannic Beyer
%   Copyright (C) 2019-2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% node force vector due to gravity
[~,q_gravity] = structureGetGravityLoadVector( m_nodes, cg_nodes, g, ...
    M_bg, structure_red.modal.T );

% add gravity force vector to external force vector
q = q + q_gravity;

% apply "practical" mean axis constraints (see [1] and [2], sec. A.2)
eta(1:6) = 0;
eta_dt(1:6) = 0;

% rigid body acceleration
[Omega_Kb_dt, V_Kb_dt] = rigidBodyKinetics( R_b, Q_b, m, I_b, g, M_bg, ...
    Omega_Kb, V_Kb );

[eta_dt2, ~] = structureGetAcc( structure_red, q, eta, eta_dt );

eta_from_7_dt2 = eta_dt2(7:end);

end