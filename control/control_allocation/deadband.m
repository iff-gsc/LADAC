function y = deadband(u, lb_min, lb_max, ub_min, ub_max)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

y = u;
if(u > ub_max)
   y =  ub_max;
   return;
end

if(u < lb_min)
   y =  lb_min;
    return;
end

if((u > lb_max) && (u < ub_min))
    
    if(u > (ub_min+lb_max)/2)
        y = ub_min;
    else
        y = lb_max ;
    end
end
   
end