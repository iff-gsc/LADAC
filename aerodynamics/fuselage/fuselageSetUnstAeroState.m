function fuselage = fuselageSetUnstAeroState( fuselage, alpha_unst, beta_unst ) %#codegen
% fuselageSetUnstAeroState set state.aero.unst struct in fuselage struct
%   If the fuselage is unsteady, the unsteady variables can be adjusted
%   during the simulation.
% 
% Syntax:
%   fuselage = fuselageSetUnstAeroState( fuselage, alpha_unst, beta_unst )
% 
% Inputs:
%   fuselage            fuselage struct (see fuselageInit)
%   alpha_unst          unsteady angle of attack, in rad
%   beta_unst           unsteady sideslip angle, in rad
% 
% Outputs:
%   fuselage            fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageSetAeroState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

fuselage.state.aero.unsteady.alpha(:) = alpha_unst;
fuselage.state.aero.unsteady.beta(:) = beta_unst;

end