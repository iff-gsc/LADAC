function structure_red = structureSetDamp( structure_red, D_h ) %#codegen
% structureSetDamp set reduced structure modal damping
%   For more information see [1], Section 2.3.
% 
% Syntax:
%   structure_red = structureGetAcc( structure_red, D )
% 
% Inputs:
%   structure_red   reduced-orger structure struct (see
%                   structureGetReduced)
%   D_h             mode damping vector, see [1], Eq. (8)
%                   ((h-6)x1 array, where h-6 is the number of mode shapes)
% 
% Outputs:
%   structure_red   reduced-orger structure struct (see
%                   structureGetReduced)
%   
% Literature:
%   [1] Höser, M., Böswald, M., & Govers, Y. (2015). Validating Global
%       Structural Damping Models for Dynamic Analyses. Deutsche
%       Gesellschaft für Luft-und Raumfahrt-Lilienthal-Oberth eV.
% 
% See also:
%   structureGetReduced

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 Davide Cavaliere
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_modeshapes = size(structure_red.K,2)-6;
if ~isequal([num_modeshapes,1],size(D_h(:)))
    error('Wrong dimensions of input D.');
else
    d_gen = 2 * D_h .* structure_red.modal.omega_red ...
        .* diag(structure_red.M(7:end,7:end));
    structure_red.d(7:end) = d_gen;
end

end
