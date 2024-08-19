function susp_payload = suspPayloadLoadMassParams( filename, susp_payload )
% SUSPPAYLOADLOADMASSPARAMS loads a suspended payloads body parameters
% (.body, .config) and appends them to susp_payload

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Jonas Withelm
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename)

susp_payload.body   = susp_payload_total.body;
susp_payload.config = susp_payload_total.config;

%{
susp_payload.rod.body   = rod.body;
susp_payload.rod.config = rod.config;

susp_payload.payload.body   = payload.body;
susp_payload.payload.config = payload.config;
%}

end
