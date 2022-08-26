function [V_Kg_dt] = ndiPlaneOmega2Acc( V_K, Omega_Kb, M_kg )

V_Kg_dt = M_kg'*[V_K_dt;0;0] + cross(Omega_Kb,[V_K;0;0]);

end