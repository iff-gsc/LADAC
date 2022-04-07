function [] = controlAllocationIndiWlsHlcBlockInit(is_Delta_W_v_checked,...
    is_W_checked,is_iter_checked)
%controlAllocationIndiWlsHlcBlockInit init block "INDI high level wls control allocation for copters"
%   If block check box is enabled, create new block inport or outport.
%   If block check box is disabled, delete new block inport or outport.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

blockInitToggleInport( 'Delta_diag_W_v', is_Delta_W_v_checked );

blockInitToggleOutport( 'W', is_W_checked );
blockInitToggleOutport( 'iter', is_iter_checked );

end