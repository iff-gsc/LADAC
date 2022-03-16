function [ K_V, I_0, R_i, m_Mot, S_max, I_max ] = ...
    motorGetParams( filename, motor_name )
% motorLoadParams loads motor parameters
%
%   The motors key parameter are being axtracted from a motor data base
%   containing multiple motors. According to the manufacturer the number of
%   parameters provided differs. 
%
% Syntax: [ K_V, I_0, R_i, m_Mot, S_max, I_max ] = ...
%   motorLoadParams( filename, motor_name )
%
% Inputs:
%   filename    matrix containing multiple motors and their key parameters
%               (matrix)
%   motor_name  name of the required motor (string)
%
% Outputs:
%   K_V         motor Kv value (scalar), in RPM/V
%   I_0         no load current (scalar), in A
%   R_i         internal motor resistance (scalar), in Ohm
%   m_Mot       motor weight (scalar), in kg
%   S_max       recommended maximum number of battery cells (scalar), -
%   I_max       maximum current (scalar), in A
%
% See also: motorOp

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Saving the database under a different variable
DATAp = evalin('base',filename);   
% Search and save the position of the column matching the motor name
ind = strcmp(DATAp(:,1),motor_name);   
% Saving the whole column 
DATAp = DATAp{ind,2};                           

% Since every database contains a different amount of motor parameters, the
% data vector length is standardised. Unknown entries will be filled with
% NaN
switch filename
    case 'axi_motor_db'
        DAT = DATAp([1 2 3 4 6 5]);
        
    case 'hacker_motor_db'
        DAT = [DATAp(1:3) NaN NaN DATAp(3)];
        
    case 'motocalc_db'
        DAT = [DATAp(1:3) NaN NaN DATAp(3)];
        
    case 'tetacalc_motor_db'
        DAT = [DATAp(1:4) NaN DATAp(5)];
        
    otherwise
        error('data base not found');
        
end


% Delivering and saving the motor parameters in the data vector
K_V = DAT(1);
I_0 = DAT(2);
R_i = DAT(3);
m_Mot = DAT(4)/1000;
S_max = DAT(5);
I_max = DAT(6);    
          

end



