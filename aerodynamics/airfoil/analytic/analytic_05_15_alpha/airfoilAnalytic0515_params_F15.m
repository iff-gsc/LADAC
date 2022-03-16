
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% weights vector for lift coefficient parameters (mse 1.0688e-04)
airfoil.wcl = [ ...
    2.5952    0.1456    1.5000   14.0529    1.0014   11.8113 ...
    0.1992   -0.2523   -0.2344   -4.1111   -3.1920  -20.3375   -0.0866    0.5782   -0.0118    0.5881    1.5182   12.7073    6.8490   17.3668  -12.0542  -10.0045    0.4840  -60.6020   -0.6708    0.5081    0.0930    0.4113    0.7809    0.2592   -0.1281   -1.9919    8.9488   -1.6691   -0.5299    7.9380    0.4295    0.5094   -1.2835   -0.0226    0.0110   -2.9419    0.2647    0.9981    0.3556   -0.8113    0.0145   -2.7431    0.0120    0.0463   -0.7067    0.3149    0.0009   -0.8353   -0.4059    0.6087    0.0442    0.3362    0.6041    0.0816...
    ];
% number of neurons in hidden layer
airfoil.ncl = 6;
% number of neural network outputs
airfoil.ocl = 6;


% weights vector for drag coefficient parameters (mse 1.1054e-04)
airfoil.wcd = [ ...
    0.0107    0.0002    3.1436   10.0000   23.1968    2.9986 ...
    28.2363    0.0874    5.8443    7.9962  -12.4390    0.5755   -7.5429    3.1916   -2.5972   -6.6195    1.2613   -0.3723   -0.0724    0.2054   -0.0769    0.0079   -0.0675    0.1002    0.5811    0.5192   -0.3377   -0.2786    0.1510    0.3078   -0.1180    1.6565    1.6456   -0.8402   -0.5229   -2.0547   -0.0094    5.3277    0.2531   -2.3299   -0.5448   -1.1944    0.4129   -1.1900    0.0786   -0.1226    0.3979   -0.2424    1.1965  -16.5555   -7.1870    8.1019    2.0284   12.2694    0.7634    1.5276   -0.6527   -0.2289    0.5072    0.2036...
    ];
% number of neurons in hidden layer
airfoil.ncd = 6;
% number of neural network outputs
airfoil.ocd = 6;
% weights vector for drag coefficient parameters


% weights vector for pitching moment coefficient parameters (mse
% 4.4214e-04)
airfoil.wcm  = [ ...
	0.1216    0.0431    0.1404    0.1491    1.2525 ...   
    -4.0534    6.7559    9.7761   -0.7829    2.8991   -3.2052   -6.2866    0.9466   -2.1118   15.7528   -5.3606    1.6885   -3.5817   -0.2573    2.4555   -0.9858    0.2182    0.3230   -0.7814    5.6643   -2.9768    1.2134   -1.8285    2.1204   -5.5834   -2.3578    0.1714    5.8448   -1.0324   -3.7784    2.5749    0.0070   -1.1460...
   ];
% number of neurons in hidden layer
airfoil.ncm = 4;
% number of neural network outputs
airfoil.ocm = 5;
