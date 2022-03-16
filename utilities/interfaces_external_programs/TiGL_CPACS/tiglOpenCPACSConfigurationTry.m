function tiglHandle = tiglOpenCPACSConfigurationTry( tixiHandle )
% tiglOpenCPACSConfigurationTry simply calls the function
%   tiglOpenCPACSConfiguration but throws a helpful error message if a
%   specicial error occurs.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

try
    tiglHandle = tiglOpenCPACSConfiguration( tixiHandle, '' );
catch ME
    if (strcmp(ME.message(1:16), 'Invalid MEX-file') )
        msg = ['\n++++++++++++++++++++++++++++ Custom message ++++++++++++++++++++++++++++\n\n', ...
            'This might be due to a known bug by the TIXI installation on Windows.\n', ...
            'To fix this, please try the following:\n\n',...
            'copy <path_to_TIGL>/bin/TIGL.dll to <path_to_TIGL>/share/tigl(3)/matlab/ \n\n', ...
            '++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause( ME, causeException );
    end
    rethrow(ME)
end

end