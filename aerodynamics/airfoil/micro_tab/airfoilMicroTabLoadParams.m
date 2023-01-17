function micro_tab = airfoilMicroTabLoadParams( filename )
% airfoilMicroTabLoadParams loads a parameter struct for a micro-tab
% 
% Example:
%   airfoil = airfoilMicroTabLoadParams( 'airfoilMicroTab_params_F15_90' )

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


run( filename );

micro_tab.net.wcl = mergeOutputLayer( micro_tab.net.wcl, micro_tab.net.ncl, micro_tab.net.ocl );
micro_tab.net.wcd = mergeOutputLayer( micro_tab.net.wcd, micro_tab.net.ncd, micro_tab.net.ocd );
micro_tab.net.wcm = mergeOutputLayer( micro_tab.net.wcm, micro_tab.net.ncm, micro_tab.net.ocm );

end

function wcl = mergeOutputLayer( weights, numNeurons, numOutputs )

idx1 = numOutputs;
idx2 = idx1 + numNeurons*2;
weights1 = reshape( weights(idx1+1:idx2), [], 2 );
weights2 = diag( weights(1:idx1) ) * reshape( weights(idx2+1:end), [], numNeurons+1 );
wcl.weights1 = weights1;
wcl.weights2 = weights2;

end