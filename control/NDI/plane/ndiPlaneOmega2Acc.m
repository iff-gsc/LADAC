function [V_Kg_dt] = ndiPlaneOmega2Acc( V_K, Omega_Kb, M_kg )


% assumption: Omega_K = Omega_gb = Omega_gk + Omega_kb ~ Omega_gk
% because Omega_kb ~ [0;0;0] because alpha_K and beta_K are approx. const.
Omega_gk_b = Omega_Kb;
V_Kg_dt = M_kg'*[V_K_dt;0;0] + cross(Omega_gk_b,[V_K;0;0]);

end