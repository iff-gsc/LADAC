function new_struct = loadLoggedData(filename, old_struct, sub_struct_name)
% **This function loads the relevant data that variated with the current
% chosen parameter**
%
%   Inputs:
%       filename:           The name of the log file 
%       old_struct:         A Matlab struct that stores the data under the given
%                           sub_struct_name
%       sub_struct_name:    The name where the derivatives and coefficients will be stored at
%
%   Process:
%       - call this function after every simulink simulation 
%       - the returned struct will be the next old struct.
%       - adjust the name according to the test case
%
%   Important:
%       - The given coefficient & derivative names in the simuling block
%         have to include the original paramater that variated e.g.:    
%               * variable: Ω_x  --> derivative: d(C_L)/d(Ω_x)
%       - The algorithim searches for the variable name in the derivative
%         name
%       - additional coefficients that do not include the variables name
%         can be added to the additional coefficients (add_coeff)
%       - the variable will be added automtically; for this the algorithm
%         searches for a variable in Testcase_variables that changed over
%         time
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


new_struct = old_struct;
Test_log = load(filename)

log_variable_names = getElementNames(Test_log.logsout);
% Testcase variables: THESE CANT BE USED IN A STRUCT DUE TO GREEK LETTERS
%   - alpha
%   - beta
%   - Ω_x
%   - Ω_y
%   - Ω_z
%   - η_asym
%   - η_sym
Testcase_variables = {'alpha', 'beta', 'Ω_x', 'Ω_y', 'Ω_z', 'η_asym', 'η_sym' }; % named in simulink (=variables)

% additional coefficients
add_coeff = {'C_D','C_Y', 'C_L' ,'C_l','C_m', 'C_n' };

orig_variable_name ='';
index_list = getIndexesOfVariables(Testcase_variables, log_variable_names);

for i=1:length(index_list)
    data_points = Test_log.logsout{index_list(i)}.Values.Data;
    delta = abs(data_points(length(data_points)) - data_points(length(data_points)-1))
    if delta > 0
        orig_variable_name = Testcase_variables{i}
        index = index_list(i);
        break;
    end
end
variable_name = renameOriginalVariableName(orig_variable_name)

% add this variable to the struct
new_struct.(sub_struct_name).(variable_name) = Test_log.logsout{index}.Values.Data    

% add all values that changed with this variable
new_struct = addChangingVariables(add_coeff, orig_variable_name, Test_log, Testcase_variables ,new_struct, sub_struct_name);

end



function new_struct = addChangingVariables(add_coeff, orig_variable_name, Test_log, TestcaseNames ,old_struct, sub_struct_name)
    % ** This function adds all the derivatives that include the name of
    %    the variable + all additional coefficients **

    new_struct = old_struct;
    %add additional coefficients
    for i=1:length(add_coeff)
        [~, index] = search4Variable([add_coeff(i)],getElementNames(Test_log.logsout));
        if isnan(index)
            continue;
        end
        %variable_name = renameOriginalVariableName(orig_variable_name);
        new_struct.(sub_struct_name).(add_coeff{i}) = Test_log.logsout{index}.Values.Data;
    end
    
    % search for according coefficients and derivatives 
    all_available_variabales = getElementNames(Test_log.logsout);
    for i=1:length(all_available_variabales)
        if contains(all_available_variabales{i},orig_variable_name)
            if Check4Variable(all_available_variabales{i}, TestcaseNames)
                coeff_name = all_available_variabales{i};
                new_coeff_name = renameCoeff(coeff_name, all_available_variabales);
                new_coeff_name = renameVariable4Struct(new_coeff_name);
                new_struct.(sub_struct_name).(new_coeff_name) = Test_log.logsout{i}.Values.Data;
            end
        end
    end
end

function isNotVariable = Check4Variable(var_name, TestcaseNames)
% ** Checks if the currently selected variable is a original variable e.g.: Ω_x **
    isNotVariable = true;
    for i =1:length(TestcaseNames)
       if contains( TestcaseNames{i}, var_name)
           isNotVariable = false;
       end
    end
end

function [variable_name, index] = search4Variable(Testcases, log_variable_names)

variable_name = '';
index = nan;
    for i=1:length(Testcases)
        for j=1:length(log_variable_names)
            if strcmp(Testcases(i), log_variable_names(j))
                variable_name = log_variable_names(j);
                index = j;
            end
        end 
    end

end

function file_save_name = renameVariable4Struct(var_name)
unexcepted_names = {'Ω', 'η'};
excepted_name = {'omega', 'eta'};


unexcepted = false;
index = 0;
for i=1:length(unexcepted_names)
    if contains( var_name, unexcepted_names{i} )
        unexcepted = true;
        index = i;
    end
    
end

if unexcepted == true
   file_save_name = replace(var_name, unexcepted_names{index}, excepted_name{index});
else
   file_save_name = var_name;
end
    

end

function new_name = renameOriginalVariableName(old_name)
% Testcase variables: THESE CANT BE USED IN A STRUCT DUE TO GREEK LETTERS
%   - alpha
%   - beta
%   - Ω_x
%   - Ω_y
%   - Ω_z
%   - η_asym
%   - η_sym
Testcase_variables = {'alpha', 'beta', 'Ω_x', 'Ω_y', 'Ω_z', 'η_asym', 'η_sym' }; % in simulink
rename_testcases = {'alpha', 'beta', 'omega_x', 'omega_y', 'omega_z', 'eta_asym', 'eta_sym'};

Name = containers.Map(Testcase_variables, rename_testcases);
new_name = Name(old_name);

end

function new_coeff_name = renameCoeff(old_name, all_orginal_names)
new_name_list = all_orginal_names;
for i=1:length(all_orginal_names)
    % example 
    % old name: d(C_L)/d(alpha) %here error for string '/' etc.
    % new name: d_C_L__d_alpha
    new_name_list{i} = replace(new_name_list{i}, '(', '_');
    new_name_list{i} = replace(new_name_list{i}, ')', '');
    new_name_list{i} = replace(new_name_list{i}, '/', '__');
    
end

M = containers.Map(all_orginal_names,new_name_list );
new_coeff_name =  M(old_name);

end

function index_list = getIndexesOfVariables(Testcase_variables, log_variable_names)
index_list = [];
for i=1:length(Testcase_variables)
   for j=1:length(log_variable_names)
     if strcmp(Testcase_variables{i}, log_variable_names{j})
         index_list(length(index_list) +1) = j;
     end
   end
    
end
end

