function wingPlotEigenmode(wing,structure,num_eigenmode,varargin)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~wing.config.is_flexible
    error('This is not a flexible wing.')
end

scaling = 1;
for i = 1:length(varargin)
    if strcmp(varargin{i},'Scaling')
        scaling = varargin{i+1};
    end
end

modal_state = pinv(structure.modal.T) * structure.modal.T(:,6+num_eigenmode) * scaling;
wing_deform = wingSetGeometryState( wing, 'pos', modal_state );
wing_deform.geometry = wing_deform.state.geometry;

wingPlotGeometry( wing, 'FaceColor', [1, 1, 1], 'FlapFaceColor', [0.9, 0.9, 0.9], ...
    'FaceAlpha', 0, 'LineColor', [0.5,0.5,0.5], 'CntrlPts', 'off', 'Faces', 'off' );

hold on

wingPlotGeometry( wing_deform, varargin{:} );

end