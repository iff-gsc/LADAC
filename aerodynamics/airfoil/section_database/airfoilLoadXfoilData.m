function data = airfoilLoadXfoilData( filename )
% airfoilLoadXfoilData loads Xfoil data into a struct.
% 
% Inputs:
%   filename        txt file name with or without '.txt' (string)
% 
% Outputs:
%   data            struct with the following fields
%                       - alpha: angle of attack in degree (array)
%                       - c_L: lift coefficient (array)
%                       - c_D: drag coefficient (array)
%                       - c_m: pitching moment coefficient (array)
%                       - info: struct containing info about Mach number,
%                           Reynolds number and Ncrit
% 
% Example:
%   First download txt file for NACA 63-412 from
%       http://airfoiltools.com/polar/details?polar=xf-n63412-il-1000000
%   Then in Matlab:
%   NACA_63_412 = airfoilLoadXfoilData( 'xf-n63412-il-1000000' );
%   plot( NACA_63_412.alpha, NACA_63_412.c_L, 'x' )
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~contains( filename, '.' )
    filename = [ filename, '.txt' ];
end

str = fileread( filename );

idx_line_end = strfind( str, newline );
num_lines = length( idx_line_end );

% parse lines until the header is found
for current_line_idx = 1:num_lines
    
    if current_line_idx == 1
        current_idx = 1;
    else
        current_idx = idx_line_end(current_line_idx-1)+1;
    end
    current_line_content = str(current_idx:idx_line_end(current_line_idx));
    
    % here the parameters are found (Mach number, Reynolds number, Ncrit)
    if contains(current_line_content,'Mach =')
        % remove spaces
        current_line_content(current_line_content == ' ') = [];
        % remove 'Mach='
        current_line_content_split = strsplit(current_line_content,'Mach=');
        % remove 'Re='
        current_line_content_split = strsplit(current_line_content_split{2},'Re=');
        data.info.Mach = str2double(current_line_content_split{1});
        % remove 'Ncrit='
        current_line_content_split = strsplit(current_line_content_split{2},'Ncrit=');
        data.info.Re = str2double(current_line_content_split{1});
        data.info.Ncrit = str2double(current_line_content_split{2}(1));
    end

    % here the header is found
    if contains(current_line_content,'---')
                  
        % read data
        startRow = 1;
        formatSpec = '%8s%9s%10s%10s%9s%9s%s%[^\n\r]';
        dataStr = [ ...
            str(idx_line_end(current_line_idx-2)+1:idx_line_end(current_line_idx-1)), ...
            str(idx_line_end(current_line_idx)+1:end) ];
        dataArray = textscan(dataStr, formatSpec, 'Delimiter', '', ...
            'WhiteSpace', '', 'TextType', 'string', 'HeaderLines', ...
            startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
        
        % only extract the required columns (alpha, CL, CD, CM)
        for i = 1:length(dataArray)
            if contains(dataArray{i}(1),'alpha')
                data.alpha = str2double(dataArray{i}(2:end))';
            elseif contains(dataArray{i}(1),'CL')
                data.c_L = str2double(dataArray{i}(2:end))';
            elseif contains(dataArray{i}(1),'CD')
                data.c_D = str2double(dataArray{i}(2:end))';
            elseif contains(dataArray{i}(1),'CM')
                data.c_m = str2double(dataArray{i}(2:end))';
            end
        end
        
        break;
                
    end

end