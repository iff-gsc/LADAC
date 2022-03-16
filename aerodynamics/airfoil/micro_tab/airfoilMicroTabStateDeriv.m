function state_dt = airfoilMicroTabStateDeriv( micro_tab, state, input )
% airfoilMicroTabStateDeriv compute state derivative for a micro-tab [1]
%   It is assumed that the micro-tab dynamics behave like a first order low
%   pass filter (T1 filter).
% 
% Inputs:
%   micro_tab           micro-tab struct (see airfoilMicroTabCreate)
%   state               state of the spoiler tab (1xN array for N spoilers;
%                       the arrays inside circulation must also have N
%                       elements)
%   input               current position of the micro-tab (1xN array);
%                       heigth relative to the chord, in percent
% 
% Outputs:
%   state_dt            time derivative of state
% 
% See also:
%   airfoilMicroTabLoadParams, airfoilMicroTabDeltaCoeff
% 
% Literature:
%   [1] Khalil, K., Asaro, S., & Bauknecht, A. (2021). Active flow control
%       devices for wing load alleviation. Journal of Aircraft, 1-17.
%       https://arc.aiaa.org/doi/pdf/10.2514/1.C036426?casa_token=ev6VDwEIkCIAAAAA:4g1h23YzMzEHPzJskme5LC45Gs0FkqwqlveAhscZVpJXFK0Do3Mh9TEMuuIvwbp9SPNOZkFzlA
%   
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% this is first order delay (T1)
state_dt = 1/micro_tab.T * ( input - state );

end