function n_dt = leanVectorNormDeriv(nn,nn_dt)
% n = nn/sqrt(nnx^2+nny^2+nnz^2) --> n_dt = ?
x=nn(1); y=nn(2); z=nn(3);
xs=nn_dt(1); ys=nn_dt(2); zs=nn_dt(3);
den = ( x^2 + y^2 + z^2 )^(3/2);
nn_dn = divideFinite( 1, den ) * ...
        [ ...
        y^2+z^2, -x*y, -x*z; ...
        -x*y, x^2+z^2, -y*z; ...
        -x*z, -y*z, x^2+y^2 ...
        ];
n_dt = nn_dn * nn_dt;

n_dt(3) = divideFinite( 1, den ) * (-x*z*xs+x^2*zs+y*(y*zs-z*ys));

end