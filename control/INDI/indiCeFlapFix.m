function [G10,G20,G30] = indiCeFlapFix( cef, ceb )

I_b = [ ...
        ceb.ixx,    -ceb.ixy,	-ceb.ixz; ...
        -ceb.ixy,	ceb.iyy,	-ceb.iyz; ...
        -ceb.ixz,	-ceb.iyz,	ceb.izz ...
    ];

inv_I_b = inv(I_b);


clu = cef.cla .* cef.dadf .* cef.dfdu;


force_dir = [ zeros(size(cef.rotx)); sin(cef.rotx); -cos(cef.rotx) ];
c_XYZ = zeros(size(force_dir));
for i = 1:size(c_XYZ,1)
    c_XYZ(i,:) = force_dir(i,:) .* clu .* cef.s;
end

pos = [ cef.x; cef.y; cef.z ];

G10 = [ ...
        inv_I_b * cross( pos, c_XYZ ); ...
        c_XYZ / ceb.m ...
    ];

G20 = [ ...
        zeros(size(G10)) ...
    ];

G30 = [ ...
        inv_I_b * cross( pos, force_dir ) * diag( cef.m .* cef.dfdu .* cef.xm ); ...
        zeros(3,size(G10,2)) ...
    ];

end