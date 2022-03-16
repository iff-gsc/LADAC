function tixiHandle = tixiOpenDocumentTry( CPACS_file_path )
% tiglOpenDocumentTry simply calls the function
%   tiglOpenDocument but throws a helpful error message if a
%   specicial error occurs.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

try
    tixiHandle = tixiOpenDocument( CPACS_file_path );
catch ME
    if (strcmp(ME.message(1:16), 'Invalid MEX-file') )
        msg = ['\n++++++++++++++++++++++++++++ Custom message ++++++++++++++++++++++++++++\n\n', ...
            'This might be due to a known bug by the TIXI installation on Windows.\n', ...
            'To fix this, please try the following:\n\n',...
            'copy <path_to_TIXI>/bin/TIXI.dll to <path_to_TIXI>/share/tixi(3)/matlab/ \n\n', ...
            '++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause( ME, causeException );
    end
    rethrow(ME)
end

end