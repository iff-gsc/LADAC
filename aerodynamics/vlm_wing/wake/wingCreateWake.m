function wake = wingCreateWake( params, geometry, n_trail )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
% *************************************************************************

wake.pos_x      = linspace( 0, 5*params.b, n_trail );
wake.pos_start  = geometry.vortex.pos(:,:,end);

end

