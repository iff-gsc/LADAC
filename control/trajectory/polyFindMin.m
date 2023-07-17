function [x_final, min_error, iters] = polyFindMin(fun, a_bnd, b_bnd)
% Define the function to be minimized
%fun = @(x) x^4 - 3*x^3 + 2;

% Define the golden ratio
phi = 1.618033988749895;

% Define the initial interval
a = a_bnd;
b = b_bnd;

fa = 0;
fb = 0;

% Define the tolerance
tol = 1e-6;

% Start the loop and update the loop counter
for iters = 1:100
    
    % Calculate the new points
    x1 = b - (b-a)/phi;
    x2 = a + (b-a)/phi;
    f1 = fun(x1);
    f2 = fun(x2);
    
    % Update the interval
    if f1 < f2
        b = x2;
    else
        a = x1;
    end
    
    if (b-a) < tol
        break;
    end
    
end

% fa = fun(a);
% fb = fun(b);
% 
% % Start the loop and update the loop counter
% for iters = 1:100
%     
%     % Quadratic Interpolation
%     x1 = a;
%     x2 = 0.5*(a+b);
%     x3 = b;
%     
%     f1 = fun(x1);
%     f2 = fun(x2);
%     f3 = fun(x3);
%     
%     x_min = (f1*x2^2 - f2*x1^2 - f1*x3^2 + f3*x1^2 + f2*x3^2 - f3*x2^2)/(2*(f1*x2 - f2*x1 - f1*x3 + f3*x1 + f2*x3 - f3*x2));
%     x_min = min(max(x_min, a_bnd), b_bnd);
%     f_min = fun(x_min);
%     
%     %disp([iters, a,x_min,b])
%     
%     if(f_min <= f1) && (f_min <= f2)
%         
%         % Update the interval based on the quadratic interpolation
%         if fa < fb
%             b  = x_min;
%             fb = f_min;
%         else
%             a  = x_min;
%             fa = f_min;
%         end
%     else
%         
%         % Calculate the new points
%         x1 = b - (b-a)/phi;
%         x2 = a + (b-a)/phi;
%         f1 = fun(x1);
%         f2 = fun(x2);
% 
%         % Update the interval
%         if f1 < f2
%             b = x2;
%         else
%             a = x1;
%         end
%         
%         
%     end
%     
%     if abs(b-a) < tol
%         break;
%     end
%     
%     
% end

a = min(max(a, a_bnd), b_bnd);
b = min(max(b, a_bnd), b_bnd);

% Calculate the solution
x_final = 0.5*(a+b);
min_error = fun(x_final);

end

