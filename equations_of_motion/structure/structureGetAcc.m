function [eta_dt2,q_elastic] = structureGetAcc( structure_red, q, eta ) %#codegen
% structureGetAcc compute acceleration of generalized load vector of a
% structural dynamics model
% 
% Syntax:
%   [eta_dt2,q_elastic] = structureGetAcc( structure_red, q, eta )
% 
% Inputs:
%   structure_red   reduced-orger structure struct (see
%                   structureGetReduced)
%   q               generalized external force vector excluding gravity
%                   (Nx1 array)
%   eta             generalized displacement vector (Nx1 array)
% 
% Outputs:
%   eta_dt2         second time derivative of input eta (Nx1 array)
%   q_elastic       elastic generalized (internal) force vector (Nx1 array)
%   
% Literature:
%   [1] Reschke, C. (2006). Integrated flight loads modelling and analysis
%       for flexible transport aircraft.
% 
% See also:
%   structureGetReduced

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [1], eq. (3.1)
q_elastic = structure_red.K*eta;
eta_dt2 = structure_red.M \ (q-q_elastic);

end
