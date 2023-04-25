function wingPlotEigenmode(wing,structure,num_eigenmode,varargin)

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~wing.config.is_flexible
    error('This is not a flexible wing.')
end

if ~isempty(varargin)
    factor = varargin{1};
else
    factor = 1;
end

modal_state = pinv(structure.modal.T) * structure.modal.T(:,6+num_eigenmode) * factor;
wing_deform = wingSetGeometryState( wing, 'pos', modal_state );
wing_deform.geometry = wing_deform.state.geometry;

wingPlotGeometry( wing, 3, [0.9, 0.9, 0.9] );

hold on

wingPlotGeometry( wing_deform, 3, 1 );

end