function pertubStruct = createStructFromBusObj( busObject, defaultValue ) %#codegen
% createStructFromBusObj creates a struct with the same fields as a bus 
%   object defined in the MATLAB Workspace. The default value of the fields
%   is defined by the second function input. The created struct can be used
%   to initialize values of a bus in Simulink.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

numElements = length( busObject.Elements );
for i = 1:numElements
    field = busObject.Elements(i).Name;
    dim = busObject.Elements(i).Dimensions;
    value = defaultValue*ones( dim, 1 );
    if i == 1
        pertubStruct = struct( field, value );
    else
        pertubStruct = setfield( pertubStruct, field, value );
    end
end