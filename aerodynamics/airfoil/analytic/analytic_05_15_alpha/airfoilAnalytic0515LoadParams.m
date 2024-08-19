function airfoil = airfoilAnalytic0515LoadParams( filename )
% airfoilAnalytic0515LoadParams loads a parameter struct for an analytic
% airfoil computation.
% 
% Example:
%   airfoil = airfoilAnalytic0515LoadParams( 'airfoilAnalytic0515_params_F15' )

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run( filename );

airfoil.wcl = mergeOutputLayer( airfoil.wcl, airfoil.ncl, airfoil.ocl );
airfoil.wcd = mergeOutputLayer( airfoil.wcd, airfoil.ncd, airfoil.ocd );
airfoil.wcm = mergeOutputLayer( airfoil.wcm, airfoil.ncm, airfoil.ocm );

end

function wcl = mergeOutputLayer( weights, numNeurons, numOutputs )

idx1 = numOutputs;
idx2 = idx1 + numNeurons*2;
weights1 = reshape( weights(idx1+1:idx2), [], 2 );
weights2 = diag( weights(1:idx1) ) * reshape( weights(idx2+1:end), [], numNeurons+1 );
wcl.weights1 = weights1;
wcl.weights2 = weights2;

end