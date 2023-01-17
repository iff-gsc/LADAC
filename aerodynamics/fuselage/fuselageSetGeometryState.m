function fuselage = fuselageSetGeometryState( fuselage, varargin ) %#codegen
% fuselageSetGeometryState set state.geometry struct in fuselage struct
%   If the fuselage is flexible, the geometry can be adjusted during the
%   simulation. Not only the position of the nodes are changed but also the
%   velocity and the acceleration are computed based on the structure
%   state.
% 
% Syntax:
%   fuselage = fuselageSetGeometryState( fuselage, modal_pos_state )
%   fuselage = fuselageSetGeometryState( fuselage, modal_pos_state, ...
%       'structure_vel', modal_vel_state )
%   fuselage = fuselageSetGeometryState( fuselage, modal_pos_state, ...
%       'structure_vel', modal_vel_state, 'structure_accel', modal_accel )
% 
% Inputs:
%   fuselage            fuselage struct (see fuselageInit)
%   modal_pos_state     structure deformation vector in modal coordinates
%   modal_vel_sate      structure velocity vector in modal coordinates
%   modal_accel         structure acceleration vector in modal coordinates
% 
% Outputs:
%   fuselage            fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageCreate, structureCreateFromNastran
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

for i = 1:length(varargin)
    if strcmp(varargin{i},'pos')
        % assign to struct
        Delta_pos = fuselage.aeroelasticity.T_cs * varargin{i+1};
        fuselage.state.geometry.cntrl_pos(:) = ...
            fuselage.geometry.cntrl_pos(:) + Delta_pos;
        fuselage.state.geometry.border_pos(:) = ...
            fuselage.geometry.border_pos(:) + ...
            ( [ 2*Delta_pos(1:3);Delta_pos] + [ Delta_pos;2*Delta_pos(end-2:end) ] )/2;
        fuselage.state.geometry.alpha(:) = fuselage.geometry.alpha + ...
            (fuselage.aeroelasticity.T_as * varargin{i+1})';
        fuselage.state.geometry.beta(:) = fuselage.geometry.beta + ...
            (fuselage.aeroelasticity.T_Bs * varargin{i+1})';
    elseif strcmp(varargin{i},'vel')
        % to do: derivative of alpha and beta
        % time derivative of displacements
        cntrl_pos_dt = reshape( fuselage.aeroelasticity.T_cs * varargin{i+1}, 3, [] );
        fuselage.state.geometry_deriv.border_pos_dt(:) = [ cntrl_pos_dt(:,1), ...
        cntrl_pos_dt(:,1:end-1) + diff(cntrl_pos_dt,1,2), cntrl_pos_dt(:,end) ];
    end
end

end