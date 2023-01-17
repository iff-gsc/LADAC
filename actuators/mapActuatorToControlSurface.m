function [ControlSurfacePosition] = mapActuatorToControlSurface ...
(ActuatorPosition, inverse_calc, ActuatorRefPos, ControlSurfaceRefPos)

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if(inverse_calc)
    ControlSurfacePosition = interp1(ControlSurfaceRefPos, ...
    ActuatorRefPos, ActuatorPosition, 'linear');
else
    ControlSurfacePosition = interp1(ActuatorRefPos, ...
    ControlSurfaceRefPos, ActuatorPosition, 'linear');
end

end

