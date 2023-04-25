function body = rigidBodyCreate()

body.V_Kb_dt        = zeros(3,1);
body.omega_Kb_dt    = zeros(3,1);
body.omega_Kb       = zeros(3,1);
body.q_bg_dt        = zeros(4,1);
body.q_bg           = [1;zeros(3,1)];
body.EulerAngles    = zeros(3,1);
body.s_g_dt         = zeros(3,1);
body.s_g            = zeros(3,1);
body.M_bg           = eye(3);
body.V_Kg           = zeros(3,1);
body.V_Kb           = zeros(3,1);

end