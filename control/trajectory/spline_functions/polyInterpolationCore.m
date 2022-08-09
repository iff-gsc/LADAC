function [x, i] = polyInterpolationCore(points, degree, cycle)


%buvwx = zeros(600,5);
%AnormAlfaRhoPhi = zeros(4,1);

%b = zeros(6*6,1);
[b, num_of_splines] = polyInterpolationb(points, degree, cycle);
A = @(x, a) polyInterpolationAx(num_of_splines, degree, cycle, x, a);
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