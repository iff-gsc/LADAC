function [G10,Gdw0,G30] = indiCeFlapFix( cef, ceb, eig )

I_b = [ ...
        ceb.ixx,    -ceb.ixy,	-ceb.ixz; ...
        -ceb.ixy,	ceb.iyy,	-ceb.iyz; ...
        -ceb.ixz,	-ceb.iyz,	ceb.izz ...
    ];

inv_I_b = inv(I_b);

force_dir = [ zeros(size(cef.rotx)); sin(cef.rotx); -cos(cef.rotx) ];
c_XYZ = zeros(size(force_dir),class(cef.clu));
for i = 1:size(c_XYZ,1)
    c_XYZ(i,:) = force_dir(i,:) .* cef.clu .* cef.s;
end

pos = [ cef.x; cef.y; cef.z ];

G10 = [ ...
        inv_I_b * cross( pos, c_XYZ ); ...
        c_XYZ / ceb.m ...
    ];

% Flap effectiveness due to downwash on horizontal tailplane
Gdw0 = zeros(size(G10));
a_z_du_dw = eig.cla_h/cef.clu(end-1) * G10(6,end-1) * eig.dahdu(:)';
q_dt_du_dw =  ceb.m/ceb.iyy*eig.x_h*a_z_du_dw;
Gdw0(2,1:length(q_dt_du_dw)) = Gdw0(2,1:length(q_dt_du_dw)) + q_dt_du_dw;
Gdw0(6,1:length(a_z_du_dw)) = Gdw0(6,1:length(a_z_du_dw)) + a_z_du_dw;


if nargout > 2
    G30 = [ ...
            inv_I_b * cross( pos, force_dir ) * diag( cef.m .* cef.dfdu .* cef.xm ); ...
            zeros(3,size(G10,2),class(G10)) ...
        ];
end

end