% Micro-tab parameters for the F15 airfoil at 90% chord
% https://arc.aiaa.org/doi/pdf/10.2514/1.C036426?casa_token=ev6VDwEIkCIAAAAA:4g1h23YzMzEHPzJskme5LC45Gs0FkqwqlveAhscZVpJXFK0Do3Mh9TEMuuIvwbp9SPNOZkFzlA

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% (mse 4.3832e-04)
micro_tab.net.wcl = [ ...
    0.3313    7.2474    2.4642 ...
    -9.7406    1.9988    7.8982    0.1989    0.6123    1.0662    2.3133   -0.4488    0.2379    1.6244   -0.9294   -0.2417   -2.6406
    ];
% number of neurons in hidden layer
micro_tab.net.ncl = 2;
% number of neural network outputs
micro_tab.net.ocl = 3;

% (mse 2.8475e-04)
micro_tab.net.wcd = [ ...
    0.0087    5.7688    3.1155 ...
    2.2364    6.3288   -0.0558   -5.2943    0.4331   -0.2857    1.1895   -0.4562   -1.1109   -1.3625    0.2212   -0.0123   -1.2419
    ];
% number of neurons in hidden layer
micro_tab.net.ncd = 2;
% number of neural network outputs
micro_tab.net.ocd = 3;

% (mse 1.0105e-04)
micro_tab.net.wcm = [ ...
    0.0556    5.8317    1.8548 ...
    -0.0940   13.5757   -0.0534  -10.7153   -3.7661   -0.0519   -1.0440    0.5462   -0.6083   -1.5800    0.8298    0.3863   -0.6661
    ];
% number of neurons in hidden layer
micro_tab.net.ncm = 2;
% number of neural network outputs
micro_tab.net.ocm = 3;

% time constant, in s
micro_tab.T = 0.068;
