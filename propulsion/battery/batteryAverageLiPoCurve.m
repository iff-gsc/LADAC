function [ DoD_max, DoD_exp, DoD_nom, V_full, V_exp, V_nom, C_rate, ...
    R_times_C_times_C_rate ] = batteryAverageLiPoCurve( )
% batteryAverageParams   creates average values for a standardized LiPo 
%   battery cell
%   The characteristic points of a LiPo discharge curve are averaged using
%   a database. The three characteristic points (full, nom and exp are
%   described in Ref. [1]. The database comes from Ref. [2].
%   For a more generic model, the absolute capacity is replaced by the
%   depth of discharge (DoC) and the absolute voltage is replaced by the
%   cell voltage.
%   The measuring points were determined at a discharge rate of 10 1/h [1].
%
% Literature:
%   [1] Tremblay, O., & Dessaint, L. A. (2009). Experimental validation of
%       a battery dynamic model for EV applications. World electric vehicle
%       journal, 3(2), 289-298.
%   [2] Gerd Giese. Elektromodellflug - Datenbank. 
%       URL: https://www.elektromodellflug.de/oldpage/datenbank.htm
%       [last downloaded 20.02.2019]
%
% Syntax: 
%   [ DoC_full, DoC_nom, DoC_exp, V_full, V_exp, V_nom, C_rate ] = 
%       batteryAverageLiPoCurve( )
%
% Outputs:
%   DoC_exp     exponential depth of discharge (scalar), -
%   DoC_nom     nominal depth of discharge (scalar), -
%   V_full      cell voltage with no current applied (scalar), in V
%   V_exp       exponential cell voltage (scalar), in V
%   V_nom       nominal cell voltage (scalar), in V
%   C_rate      discharge rate (scalar), in 1/h
%
% See also: batteryDischargeParams, batteryVoltage
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2016-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load database
load('battery_lipo_database');
DATA = Elektromodellflug;

% Extract the necessary discharge points and the capacity from the data
% base
discharge_points = cell2mat(DATA(:,3));
num_series = cell2mat(DATA(:,4));
capacity = cell2mat(DATA(:,5));

% Standardising the battery capacity to the State of Charge
% The capacity values are given in As, ./3.6 transforms them into
% mAh, finally the figures are standardised with the overall battery
% capacity in mAh
DoD_max = discharge_points(:,1) ./ 3.6 ./ capacity;
DoD_nom = discharge_points(:,2) ./ 3.6 ./ capacity;
DoD_exp = discharge_points(:,3) ./ 3.6 ./ capacity;

% Voltage discharge points in V
V_full = discharge_points(:,4);
V_exp = discharge_points(:,5);
V_nom = discharge_points(:,6);

cell_internal_resistance = discharge_points(:,8);

% All points of the discharge curve have been determined at a discharge
% rate (C-Rate) of 10 1/h
C_rate = 10;

C_rating = cell2mat( DATA(:,6) );
C_rating_data = [25,30,35,40,45];
C_rating_data_SI = C_rating_data / 3600;
R_times_C_data_SI = zeros(size(C_rating_data));
capacity_SI = capacity / 1000 * 3600;

figure
hold on

C_rating_plot = [];
R_times_C_plot = [];
for i = 1:length(R_times_C_data_SI)
    cell_internal_resistance_i = cell_internal_resistance(C_rating==C_rating_data(i));
    cell_capacity_SI_i = capacity_SI(C_rating==C_rating_data(i));
    R_times_C_data_SI_i = cell_internal_resistance_i .* cell_capacity_SI_i;
    R_times_C_data_SI(i) = median( R_times_C_data_SI_i );
    % plot( C_rating_data(i), R_times_C_data_SI_i, 'bx' )
    C_rating_plot = [C_rating_plot, repmat(C_rating_data(i),1,length(R_times_C_data_SI_i))];
    R_times_C_plot = [R_times_C_plot, R_times_C_data_SI_i(:)'];
end

plot( C_rating_plot, R_times_C_plot, 'bx' )

plot( C_rating_data, R_times_C_data_SI, 'rs' )

fun = @(x,xdata)x./xdata;

x0 = 1;

R_times_C_times_C_rate = lsqcurvefit(fun,x0,C_rating_data_SI,R_times_C_data_SI,0,inf);

C_rating_test = linspace( 10, 100, 100 );
C_rating_test_SI = C_rating_test / 3600;
plot( C_rating_test, R_times_C_times_C_rate ./ C_rating_test_SI );
xlim([0,inf])
ylim([0,inf])
grid on

xlabel( 'C rating, 1/h' )
ylabel( 'Cell capacity * cell internal resistance, Vs' )

legend('Data base','Median of data base','Curve fit')

figure
hold on
capacity_test = linspace( 0.1, 10, 1000 );
capacity_test_SI = capacity_test * 3.6 * 1000;
Legend = {};
C_rating_test_2 = linspace( C_rating_test(1), C_rating_test(end), 10 );
C_rating_test_2_SI = C_rating_test_2 / 3600;
for i = 1:length(C_rating_test_2)
    cell_internal_resistance_test = R_times_C_times_C_rate ./ capacity_test_SI / C_rating_test_2_SI(i);
    semilogx( capacity_test, cell_internal_resistance_test * 1000 )
    Legend(end+1) = {['C rating: ',num2str(C_rating_test_2(i)),' 1/h']};
end
xlabel( 'Cell capacity, Ah' )
ylabel( 'Cell internal resistance, m\Omega' )
grid on
set(gca, 'XScale', 'log');
set(gca, 'YScale', 'log');

legend(Legend);

num_batteries = length(V_full);
Eo = zeros(num_batteries,1);
A = zeros(num_batteries,1);
K = zeros(num_batteries,1);
B = zeros(num_batteries,1);
for i = 1:num_batteries
    DoD_nom(DoD_nom>0.98) = 0.98;
    C_rating_SI = C_rating(i) / 3600;
    R_times_C = R_times_C_times_C_rate ./ C_rating_SI;
    DoD_nom(1) = 0.85;
    V_nom(1) = 3.5;
    [Eo(i), A(i), K(i), B(i)] = batteryParamsFromCurve( DoD_max(i), DoD_exp(i), ...
        DoD_nom(i), V_full(i), V_exp(i), V_nom(i), C_rate, R_times_C );
end

end
