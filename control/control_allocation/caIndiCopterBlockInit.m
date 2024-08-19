function [] = caIndiCopterBlockInit(is_ce_extern)
%caIndiWlsHlcBlockInit init block "INDI high level wls control allocation for copters"
%   If block check box is enabled, create new block inport or outport.
%   If block check box is disabled, delete new block inport or outport.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

blockInitToggleInport( 'G1_extern', is_ce_extern );
blockInitToggleInport( 'G2_extern', is_ce_extern );

end