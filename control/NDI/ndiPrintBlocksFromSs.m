function [] = ndiPrintBlocksFromSs( A )
% ndiPrintBlocksFromSs visualizes a transformed system with feedback
% linearization including a first-order delay to take higher order dynamics
% into account.
% 
% Syntax:
%   ndiPrintBlocksFromSs( A )
% 
% Inputs:
%   sys_order       order of the above system (number of states)
%   T_h             first-order delay time constant to account for higher
%                   order dynamics
% 
% Outputs:
%   none
% 
% Example:
%   [~,A,~] = ndiFeedbackGainLqr([0.1,1,10],3,0.015)
%   ndiPrintBlocksFromSs( A )
% 
% See also:
%   ndiOpenLoopSs, ndiFeedbackGainLqr
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% system order and first-order delay
sys_order = size(A,1);
T_h = -1/A(end,end);
is_first_order_delay = T_h > 0;

% define parameters
arrow_length = 2;
integrator_width = 7;
T_h_str = num2str(T_h);
t1_width = 10 + length(T_h_str);
input_name = 'Delta_ny';
output_name = 'e_y';

% integrator definition
integrator_first_row = [' ',repmat('-',1,integrator_width-2),' '];
integrator_spaces = repmat(' ',1,floor((integrator_width-2)/2));
integrator_second_row = ['|',integrator_spaces,'1',integrator_spaces,'|'];
integrator_third_row = ['| ',repmat('-',1,integrator_width-4),' |'];
integrator_fourth_row = ['|',integrator_spaces,'s',integrator_spaces,'|'];

% arrow definition
arrow_ = [repmat('-',1,arrow_length),'>'];
arrow_spaces = repmat(' ',1,arrow_length+1);

% input layer
first_row = repmat(' ',1,length(input_name)+1);
second_row = first_row;
third_row = [input_name,' '];
fourth_row = first_row;

% first-order delay
if is_first_order_delay
    
    num_integrators = sys_order - 1;
    
    t1_top = [' ',repmat('-',1,t1_width-2),' '];
    first_row = [first_row,arrow_spaces,t1_top];
    t1_space_num = repmat(' ',1,floor((t1_width-2)/2));
    if mod(t1_width,2) == 0
        t1_space_num_1 = t1_space_num(1:end-1);
    else
        t1_space_num_1 = t1_space_num;
    end
    second_row = [second_row,arrow_spaces,'|',t1_space_num_1,'1',t1_space_num,'|'];
    third_row = [third_row,arrow_,'| ',t1_top(3:end-2),' |'];
    fourth_row = [fourth_row,arrow_spaces,'| ',T_h_str,'*s + 1 |'];
    
else
    num_integrators = sys_order;
end

% integrators
for i = 1:num_integrators
    first_row = [first_row,arrow_spaces,integrator_first_row];
    second_row = [second_row,arrow_spaces,integrator_second_row];
    third_row = [third_row,arrow_,integrator_third_row];
    fourth_row = [fourth_row,arrow_spaces,integrator_fourth_row];
end

third_row = [third_row,arrow_,' ',output_name];

fifth_row = first_row;

disp(first_row);
disp(second_row);
disp(third_row);
disp(fourth_row);
disp(fifth_row);

end
