function [G10,G20] = indiCeFlapFix( cef, ceb, T_s )

I_b = [ ...
        ceb.ixx,    -ceb.ixy,	-ceb.ixz; ...
        -ceb.ixy,	ceb.iyy,	-ceb.iyz; ...
        -ceb.ixz,	-ceb.iyz,	ceb.izz ...
    ];

inv_I_b = inv(I_b);


cef.cla;

cef.dadf;

clu = cef.cla .* cef.dadf .* cef.dfdu;

cef.s;

cef.pos;

c_XYZ = [ zeros(size(cef.rotx)); sin(cef.rotx); -cos(cef.rotx) ];

for i = 1:size(c_XYZ,1)
    c_XYZ(i,:) = c_XYZ(i,:) .* clu .* cef.s;
end

G10 = [ ...
        inv_I_b * cross( cef.pos, c_XYZ ); ...
        c_XYZ / ceb.m ...
    ];

G20 = 1 / T_s * ...
    [ ...
        zeros(size(G10)) ...
    ];


G10(1,1:2) = G10(1,1:2);

end