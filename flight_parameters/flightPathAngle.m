function gamma = flightPathAngle( V_Kg ) %#codegen
% Brockhaus (2011), Eq. (1.2.8)

V_K = norm( V_Kg, 2 );
gamma = asinReal( V_Kg(3) / V_K );

end