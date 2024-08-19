function [weights,mse] = airfoilAnalytic0515NeunFit( net_settings, inputs, targets, mse_target )
% airfoilAnalytic0515NeunFit runs a neural network training of a small size
% neural network multiple times to approximate Mach number dependent
% coefficients of analytic functions in the airfoilAnalytic0515 project.
%   The neural network is supposed to compute the analytic function
%   coefficients for different Mach numbers faster then a interpolation
%   of stored data. Therefore, the neural network should have as few
%   neurons/weights as possible. Since a good approximation with a
%   critically small neural network often is not achieved (local minimum
%   is reached at a high error), the neural network weights are randomly
%   initialized multiple times to try to get a better fit again and again.
%   This function tries to predict repeatedly whether it is better to
%   continue the training of the current weights or if it is a waste of
%   time and better to try again with new weights. This prediction is done
%   by an empirical formular based on the relative error loss.
%   If the specified MSE could be reached, training is continued until the
%   training process becomes slow, then the function will return this
%   weights. If the specified MSE can not be reached repeatedly, the
%   weights with the lowest MSE are stored and returned after a certain
%   number of iterations.
% 
% Inputs:
%   net_settings        file name of a NeuN setting file (string)
%   inputs              neural network inputs (1x1xN array)
%   targets             neural network targets (Mx1xN array)
%   mse_target          the target for the MSE (scalar)
% 
% Outputs:
%   weights             neural network weights as a vector
%   mse                 the achieved MSE (scalar)
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_iter_sub = 2000;
num_init_net_max = 50;
num_repeat_iter_max = 150;

mse_best_net = 1;

for i = 1:num_init_net_max
    
    disp('*********************   N E W   N E T W O R K   *********************')

    % clear net
    clear neuralNetwork buildGradientFromDelta resilientPropagation...
    gradientDescentMomentum Adam;

    % init net
    [NET,TDATA]=createNet(net_settings);

    mse_last_iter = 1;

    for j = 1:num_repeat_iter_max

        % train net
        [NET,TDATA,MSE,fNorm] = neuralNetTraining( inputs, targets, NET, TDATA, num_iter_sub );

        mse = MSE(end);
        
        red_fac_max = 0.3;
        red_fac_mid = 0.0015;
        red_fac_min = 0.00025;
        % exponential function where...
        %    red_fac = red_fac_max for mse -> inf
        %    red_fac = red_fac_mid for mse = mse_target
        %    red_fac = red_fac_min for mse -> 0
        red_fac = ( red_fac_max- red_fac_min ) ...
            * exp( (mse_target / mse)^0.7 * log(1/(red_fac_max-red_fac_min)*(red_fac_mid-red_fac_min)) ) ...
            + red_fac_min;
%         if (mse > (1-red_fac) * mse_last_iter)
%         if ( mse + ( mse - mse_last_iter ) * num_repeat_iter_max/5 > mse_target ) || (mse>0.995*mse_last_iter)
        cond1 = ( mse * (mse/mse_last_iter)^(tanh(50*mse_target/mse)*num_repeat_iter_max) > mse_target );
        cond2 = (mse>0.995*mse_last_iter) && ~cond1;
        if cond1 || cond2
            break;
        end

        mse_last_iter = mse;

    end
    
    if mse < mse_best_net
        weightsT = scellArrT( NET.weights );
        weights = weightsT.data(:)';
        
        mse_best_net = mse;
    end
    
    
    if (mse_best_net < mse_target)
        break;
    end
    
end

mse = mse_best_net;

end

