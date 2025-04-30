function structure = structureCreate( filename )
% structureCreate create of a structural dynamics beam-element model from
%   parameter file
% 
% Syntax:
%   structure = structureCreate( filename )
% 
% Inputs:
%   filename        Name of the parameters file as defined by
%                   structure_params_default.m
% 
% Outputs:
%   structure       Structural dynamics struct (as defined by this
%                   function) with the following fields:
%                   - xyz: Nodes positions, m (3xN matrix, where N is the
%                       number of nodes,
%                   - K: Stiffness matrix (6*Nx6*N matrix), SI units
%                   - M: Mass matrix (6*Nx6*N matrix), SI units
% 
% Example:
%   structure = structureCreate( 'structure_params_default' );
%   figure
%   structurePlot( structure )
%   figure
%   structurePlotEigenmode( structure, 1, 'Scaling', 0.2 )
% 
% See also:
%   structurePlot, structurePlotEigenmode, structureGetReduced
% 
% Literature:
%   [1] Andrews, S. P. (2011). Modelling and simulation of flexible
%       aircraft: handling qualities with active load control. PhD thesis,
%       Cranfield University.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024-2025 Yannic Beyer
%   Copyright (C) 2024-2025 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


param = loadParams( filename );

xyz = param.xyz;

num_nodes = size(xyz,2);
dof1 = 6;
dof = dof1*num_nodes;
num_connections = size(param.stiff,1);

structure.xyz = param.xyz;
structure.K = zeros( dof, dof );
structure.M = zeros( dof, dof );

% [1], eqs. (3.4.9) and (3.4.10)
for i = 1:num_nodes
    m = param.mass(i,1);
    mx = param.mass(i,2);
    my = param.mass(i,3);
    mz = param.mass(i,4);
    Ixx = param.mass(i,5);
    Iyy = param.mass(i,6);
    Izz = param.mass(i,7);
    Ixy = param.mass(i,8);
    Ixz = param.mass(i,9);
    Iyz = param.mass(i,10);
    Mxyz = [ 0, mz, -my; mz, 0, -mx; my, -mx, 0 ];
    Ixyz = [ Ixx, -Ixy, -Ixz; -Ixy, Iyy, -Iyz; -Ixz, -Iyz, Izz ];
    M = [ diag( m*ones(3,1) ), Mxyz; Mxyz', Ixyz ];
    i1 = (i-1)*dof1+1:i*dof1;
    structure.M(i1,i1) = M;
end

% [1], eq. (3.4.4)
for i = 1:num_connections
    idx1 = param.stiff(i,1);
    idx2 = param.stiff(i,2);
    A = param.stiff(i,3);
    Iy = param.stiff(i,4);
    Iz = param.stiff(i,5);
    E = param.stiff(i,6);
    J = param.stiff(i,7);
    G = param.stiff(i,8);
    direction = xyz(:,idx2) - xyz(:,idx1);
    L = norm(direction,2);
    x_loc = divideFinite( direction, L );
    if direction(1) == 0 && direction(2) == 0
        y_loc = [ 0; 1; 0 ];
    else
        y_loc = cross( [0;0;1], direction );
    end
    y_loc = divideFinite( y_loc, norm(y_loc,2) );
    z_loc = cross( x_loc, y_loc );
    x_glo = [1;0;0];
    y_glo = [0;1;0];
    z_glo = [0;0;1];
    T_u = [ ...
        dot(x_glo,x_loc), dot(x_glo,y_loc), dot(x_glo,z_loc); ...
        dot(y_glo,x_loc), dot(y_glo,y_loc), dot(y_glo,z_loc); ...
        dot(z_glo,x_loc), dot(z_glo,y_loc), dot(z_glo,z_loc) ...
        ];
    
    K = [ ...
        A*E/L, 0, 0, 0, 0, 0, -A*E/L, 0, 0, 0, 0, 0; ...
        0, 12*E*Iz/L^3, 0, 0, 0, 6*E*Iz/L^2, 0, -12*E*Iz/L^3, 0, 0, 0, 6*E*Iz/L^2; ...
        0, 0, 12*E*Iy/L^3, 0, -6*E*Iy/L^2, 0, 0, 0, -12*E*Iy/L^3, 0, -6*E*Iy/L^2, 0; ...
        0, 0, 0, G*J/L, 0, 0, 0, 0, 0, -G*J/L, 0, 0; ...
        0, 0, -6*E*Iy/L^2, 0, 4*E*Iy/L, 0, 0, 0, 6*E*Iy/L^2, 0, 2*E*Iy/L, 0; ...
        0, 6*E*Iz/L^2, 0, 0, 0, 4*E*Iz/L, 0, -6*E*Iz/L^2, 0, 0, 0, 2*E*Iz/L; ...
        -A*E/L, 0, 0, 0, 0, 0, A*E/L, 0, 0, 0, 0, 0; ...
        0, -12*E*Iz/L^3, 0, 0, 0, -6*E*Iz/L^2, 0, 12*E*Iz/L^3, 0, 0, 0, -6*E*Iz/L^2; ...
        0, 0, -12*E*Iy/L^3, 0, 6*E*Iy/L^2, 0, 0, 0, 12*E*Iy/L^3, 0, 6*E*Iy/L^2, 0; ...
        0, 0, 0, -G*J/L, 0, 0, 0, 0, 0, G*J/L, 0, 0; ...
        0, 0, -6*E*Iy/L^2, 0, 2*E*Iy/L, 0, 0, 0, 6*E*Iy/L^2, 0, 4*E*Iy/L, 0; ...
        0, 6*E*Iz/L^2, 0, 0, 0, 2*E*Iz/L, 0, -6*E*Iz/L^2, 0, 0, 0, 4*E*Iz/L ...
        ];
    
    T = zeros(size(K));
    for j = 1:4
        j1 = (j-1)*3+1:3*j;
        T(j1,j1) = T_u;
    end
    KT = T*K*T';
    
    i1 = (idx1-1)*dof1+1:idx1*dof1;
    i2 = (idx2-1)*dof1+1:idx2*dof1;
    structure.K(i1,i1) = structure.K(i1,i1) + KT(1:dof1,1:dof1);
    structure.K(i1,i2) = structure.K(i1,i2) + KT(1:dof1,dof1+1:2*dof1);
    structure.K(i2,i1) = structure.K(i2,i1) + KT(dof1+1:2*dof1,1:dof1);
    structure.K(i2,i2) = structure.K(i2,i2) + KT(dof1+1:2*dof1,dof1+1:2*dof1);
end
