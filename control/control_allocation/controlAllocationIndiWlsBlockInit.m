function [] = controlAllocationIndiWlsBlockInit(is_Delta_u_d_checked,...
    is_Delta_gamma_checked,is_Delta_W_v_checked,...
    is_W_checked,is_iter_checked)
%controlAllocationIndiWlsBlockInit init block "INDI wls control allocation"
%   If block check box is enabled, create new block inport or outport.
%   If block check box is disabled, delete new block inport or outport.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

blockInitToggleInport( 'Delta u_d', is_Delta_u_d_checked );
blockInitToggleInport( 'Delta gamma', is_Delta_gamma_checked );
blockInitToggleInport( 'Delta diag W_v', is_Delta_W_v_checked );

blockInitToggleOutport( 'W', is_W_checked );
blockInitToggleOutport( 'iter', is_iter_checked );

end