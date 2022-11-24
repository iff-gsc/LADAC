
% create default battery model with 4 serial cells, 4Ah, 35C, V_min=3V
bat = batteryCreate( 'battery_params_default', 4, 4, 35, 3 );

C_rate_test = [0.1,1,5,10,20,35];
Legend = {};
figure
hold on
for i = 1:length(C_rate_test)
    batteryPlotDischargeCurve(bat,C_rate_test(i));
    Legend(end+1) = {['C rating: ',num2str(C_rate_test(i)),' 1/h']};
end

grid on
legend(Legend);

%%

R_times_C_times_C_rate = 0.5128;

DoD_exp = 0.2;
DoD_nom = 0.95;
DoD_max = 1.1;
V_full  = 4;
V_exp   = 3.8;
V_nom   = 3.4;
C_rate  = 10;
C_rating = 25;

bat = batteryCreate('battery_params_default', 4, 4, C_rating, 3 );
R_times_C = R_times_C_times_C_rate / (C_rating/3600);

[bat.E_0, bat.A, bat.K, bat.B] = batteryParamsFromCurve( DoD_exp, ...
    DoD_nom, DoD_max, V_full, V_exp, V_nom, C_rate, R_times_C );
bat.SoC_full = DoD_max;
bat.R_times_C = R_times_C;
bat.V_min = 3;

figure
plot( [ 0, DoD_exp, DoD_nom ], [ V_full, V_exp, V_nom ], 'x' )
hold on
plot( [DoD_max,DoD_max], [bat.V_min,V_full], 'k--' )
batteryPlotDischargeCurve(bat,C_rate)

legend('Given data points','Given pole location','Fited model','location','southwest')
