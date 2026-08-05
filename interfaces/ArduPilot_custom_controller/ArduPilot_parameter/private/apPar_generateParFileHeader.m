function header = apPar_generateParFileHeader(model_pars, code_pars, varargin)
% APPAR_GENERATEPARFILEHEADER generates an info header in the following
%   form:
% 
%   Custom Controller Parameters
%   
%   Simulink Model Name:    <model_pars.name>
%   Simulink Model Version: <model_pars.version>
%   Code Export Version:    <code_pars.version>
%   Date:                   <time_stamp>


%% Input Parser
p = inputParser;
p.addParameter('time_stamp', datestr(datetime, 'dd-mm-yyyy HH:MM:SS'), @(a) ischar(a) || isstring(a) );
p.parse(varargin{:});

time_stamp = p.Results.time_stamp;

    
header = sprintf(['Custom Controller Parameters\n\n', ...
            'Simulink model name:    %s\n', ...
            'Simulink model version: %s\n', ...
            'Code export version:    %s\n', ...
            'Date:                   %s'], ...
            model_pars.name, model_pars.version, code_pars.version, time_stamp);

end
