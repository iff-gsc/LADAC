function n_dt2 = leanVectorNormDeriv2(nn,nn_dt,nn_dt2)
% wolframalpha: second derivative of (x(t)/sqrt(x(t)^2+y(t)^2+z(t)^2))
n_dt2 = zeros(3,1);
n_dt2(1) = leanVectorDeriv2Comp(nn,nn_dt,nn_dt2,'x');
n_dt2(2) = leanVectorDeriv2Comp(nn,nn_dt,nn_dt2,'y');
n_dt2(3) = leanVectorDeriv2Comp(nn,nn_dt,nn_dt2,'z');
    
    function n_comp_dt2 = leanVectorDeriv2Comp(nn,nn_dt,nn_dt2,comp)
        switch comp
            case 'x'
                i1=1; i2=2; i3=3;
            case 'y'
                i1=2; i2=1; i3=3;
            case 'z'
                i1=3; i2=1; i3=2;
        end
        x=nn(i1); y=nn(i2); z=nn(i3);
        xs=nn_dt(i1); ys=nn_dt(i2); zs=nn_dt(i3);
        xss=nn_dt2(i1); yss=nn_dt2(i2); zss=nn_dt2(i3);
        xyz = x^2+y^2+z^2;
        n_comp_dt2 = xss/sqrt(xyz) ...
            -(xs*(2*x*xs+2*y*ys+2*z*zs))/(xyz^(3/2)) ...
            +x*(...
                (3*(2*x*xs+2*y*ys+2*z*zs)^2)/(4*xyz^(5/2)) ...
                -(2*x*xss+2*xs^2+2*y*yss+2*ys^2+2*z*zss+2*zs^2)/(2*xyz^(3/2)) ...
            );
    end
end