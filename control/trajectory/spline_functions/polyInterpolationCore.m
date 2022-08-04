function [x, i] = polyInterpolationCore(points, degree, cycle)

b = polyInterpolationb(points, degree, cycle);

A = @(x, a) polyInterpolationAx(points, degree, cycle, x, a);

[ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ladac_lsqr_init(A, b);

for i=1:1000
    [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ...
        ladac_lsqr_iterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar);
    if( norm( A(x,1) - b ) < 1e-4)
        disp(i);
        break;
        
    end
end


end