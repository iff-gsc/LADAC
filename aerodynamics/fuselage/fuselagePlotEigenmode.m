function fuselagePlotEigenmode(fuselage,structure,num_eigenmode,varargin)
% fuselagePlotEigenmode visualize eigenmode of fuselage
% 
% Syntax:
%   fuselagePlotEigenmode(fuselage,structure,num_eigenmode)
%   fuselagePlotEigenmode(fuselage,structure,num_eigenmode,factor)
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   structure     	a structure struct (see structureCreateFromNastran)
%   num_eigenmode   number of eigenmode to be visualized
%   factor          factor to increase the deformation (default is 1)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageCreateWithCpacs
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if num_eigenmode > size(fuselage.aeroelasticity.T_cs,2)
    error('Requested eigenmode exceeds the number of eigenmodes.')
end

if ~isempty(varargin)
    factor = varargin{1};
else
    factor = 1;
end

modal_state = pinv(structure.modal.T) * structure.modal.T(:,num_eigenmode) * factor;
fuselage_deform = fuselageSetGeometryState( fuselage, modal_state );
fuselage_deform.geometry = fuselage_deform.state.geometry;

fuselagePlotGeometry( fuselage );

hold on

fuselagePlotGeometry( fuselage_deform );

end