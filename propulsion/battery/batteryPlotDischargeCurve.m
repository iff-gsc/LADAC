function [] = batteryPlotDischargeCurve(bat,C_rate)

Delta_t = 1;
SoC_sim = [];
V_cell_sim = [];
C_rate_SI = C_rate/3600;
SoC = 1;
C_rate_filt = 0;
while true
     [V_bat,V_cell] = batteryVoltage(bat,SoC,C_rate_SI,C_rate_filt);
     if SoC <= 1 - bat.SoC_full
         SoC_sim(end) = [];
         V_cell_sim(end) = [];
         break;
     else
         SoC_sim(end+1) = SoC;
         V_cell_sim(end+1) = V_cell;
         SoC = SoC - C_rate_SI * Delta_t;
         C_rate_filt = C_rate_filt + 1 / bat.tc*(C_rate_SI-C_rate_filt)*Delta_t;
     end
end
DoD_sim = 1 - SoC_sim;
plot( DoD_sim, V_cell_sim )

grid on

xlabel('Depth of discharge, dimensionless')
ylabel('Battery cell voltage, V')

end