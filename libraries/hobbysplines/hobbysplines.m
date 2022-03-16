function [Q, traj] = hobbysplines(points,varargin)
%HOBBYSPLINES Draws open or closed smooth curves from keypoints
%
% Q = hobbysplines({[x1 y1], [x2 y2],... ,[xN yN]},[opts]);
% Q = hobbysplines({[x1 y1 z1], [x2 y2 z2],... ,[xN yN zN]},[opts]);
%
% Draws an open or cyclic curve through keypoints 1 to N.
% Points are output as matrix "Q".
% Keypoints may be specified with optional slopes and tension parameters.
% The syntax to do this (replacing `[x1 y1]` above, say) is
%
% 2D:
%    { [x1 y1] s1 t1 }
%    { [x1 y1] s1 t1_in t1_out }
%    { [x1 y1] [u1 v1] ... }
%
% 3D:
%    { [x1 y1 z1] [u1 v1 w1] t1 }
%    { [x1 y1 z1] [u1 v1 w1] t1_in t1_out }
%
% where `s1` is the slope in degrees of the curve through `[x1 y1]`,
% `[u1 v1 w1]` is the unit vector describing the slope of the curve,
% and `t1_out` & `t1_in` are the "exit" and "entry" tensions of the curve
% approaching that point. If `t1` only is specified, this is both the
% entry and the exit tension.
%
% Use '' to indicate default values here; for example, for a default slope
% but specified "entry" tension of 2.0, use
%
%    { [x1 y1] '' 2.0 ''}
%
% and note that trailing optional arguments can also be omitted.
% According to Hobby, tensions should not be specified less than 0.75.
% This software does not enforce this restriction.
% Note that a tension of 1 creates approximately circular plots.
%
% Optional arguments given by [opts] can be any combination of the
% following:
%
%   OPTION       DEFAULT            DESCRIPTION
%   ------       -------            -----------
%   'tension'    [1]                default tension between points
%   'offset'     [0 0 0]            offset to add to each control point
%   'cycle'      true               whether to draw a cyclic curve
%   'debug'      false              draw and label keypoints on the curve
%   'linestyle'  {'linewidth',1}    line style option(s)
%   'color'      'black'            colour of the curve
%   'bezierpoints' 50               number of points in each bezier segment
%   'plot'       true               whether to actually draw anything
%
% Distributed under the terms and conditions of the 2-clause BSD license:  
% <http://opensource.org/licenses/bsd-license.php>
%
% Copyright 2013, 2015 Will Robertson and The University of Adelaide
% All rights reserved.

%% Parse inputs

p = inputParser;
p.addRequired('points',@iscell)
p.addOptional('tension',1);
p.addOptional('offset',[0 0]);
p.addOptional('cycle',true);
p.addOptional('color','black');
p.addOptional('debug',false);
p.addOptional('linestyle',{'linewidth',1});
p.addOptional('bezierpoints',50)
p.addOptional('plot',true);

p.parse(points,varargin{:});

cycle  = p.Results.cycle;
offset = p.Results.offset;
debug  = p.Results.debug;
points = p.Results.points;
bezierpoints = p.Results.bezierpoints;
plotcurve = p.Results.plot;

if numel(offset) == 2, offset(3) = 0; end

color = p.Results.color;
linestyle = p.Results.linestyle;
if ~iscell(linestyle)
  linestyle = {linestyle};
end
  
if cycle
  points{end+1} = points{1};
end

Npoints = numel(points);

z = cell(Npoints,1); % points
w = cell(Npoints,1); % unit vectors of direction of curve through each point
tin = cell(Npoints,1); % tension of curve in to point
tout = cell(Npoints,1); % tension of curve out from point

for n = 1:Npoints
  
  pp = points{n};
  
  w{n} = [NaN NaN NaN];
  tout{n} = p.Results.tension;
  tin{n} = p.Results.tension;
  
  if iscell(pp)
    % for input in the form pp := { [x1 y1 z1] s1 t1_in t1_out }
    
    veclen = numel(pp);
    
    if numel(pp{1}) == 2
      z{n} = offset+[pp{1} 0];
    else
      z{n} = offset+pp{1};
    end
    
    if veclen >= 2 && isnumeric(pp{2}) % lazy evaluation is my friend
      switch numel(pp{2}) 
        case 1
          w{n} = [cosd(pp{2}) sind(pp{2}) 0];
        case 2
          w{n} = [pp{2} 0];
        case 3
          w{n} = pp{2};
      end
    end
    
    if veclen >= 3 && isnumeric(pp{3})
      tin{n} = pp{3};
    end
    
    if veclen == 4 && isnumeric(pp{4})
      tout{n} = pp{4};
    end
    
  else
    % if input in the form pp := [x1 y1 z1]
    
    if numel(pp) == 2
      z{n} = offset+[pp 0];
    else
      z{n} = offset+pp;
    end
  end
  
end


%% fixup vectors iff necessary

if all( isnan(w{1}) )
  if cycle
    w{1} = z{2}-z{end-1};
  else
    w{1} = z{2}-z{1};
  end
  w{1} = w{1}/norm(w{1});
end
if all( isnan(w{end}) )
  if cycle
    w{end} = z{2}-z{end-1};
  else
    w{end} = z{end}-z{end-1};
  end
  w{end} = w{end}/norm(w{end});
end
for ii = 2:Npoints-1
  if all( isnan(w{ii}) )
    w{ii} = -z{ii-1} + z{ii+1};
  end
  w{ii} = w{ii}/norm(w{ii});
end

%% Calculate control points and plot bezier curve segments

hold on

q = nan(bezierpoints*(Npoints-1),3);

traj.num_sections_set = Npoints-1;
traj.num_sections_max = Npoints-1;
traj.active_section = 0;
traj.current_time = 0;

for ii = 1:Npoints-1
  
 %  theta = arg(w{ii})-arg(z{ii+1}-z{ii});
%   phi   = arg(z{ii+1}-z{ii})-arg(w{ii+1});

  theta = arg3d(w{ii}, z{ii+1}-z{ii});
  phi   = arg3d(z{ii+1}-z{ii}, w{ii+1});
  
  
  [rho,sigma] = velocity_parameters(theta,phi);
  
  P1 = z{ii};
  P2 = z{ii}+rho/(3*tout{ii})*norm(z{ii+1}-z{ii})*w{ii};
  P3 = z{ii+1}-sigma/(3*tin{ii+1})*norm(z{ii+1}-z{ii})*w{ii+1};
  P4 = z{ii+1};
  
  [q( (ii-1)*bezierpoints + (1:bezierpoints) ,:), coeffs] = plot_bezier(...
    P1, P2, P3, P4, ...
    bezierpoints,plotcurve,...
    [linestyle,{'color',color}]);

 traj.sections(ii).pos_x = coeffs(1,:);
 traj.sections(ii).pos_y = coeffs(2,:);
 traj.sections(ii).pos_z = coeffs(3,:);

 traj.sections(ii).t = 0;
 traj.sections(ii).vel = 0;
  
end

traj = trajSetArcLength(traj);

%%

if nargout >= 1
   Q = q; 
end

if plotcurve && debug
  if cycle
    Mpoints = Npoints-1;
  else
    Mpoints = Npoints;
  end
  for ii = 1:Mpoints
    plot3(z{ii}(1),z{ii}(2),z{ii}(3),'o','color',color)
    text(z{ii}(1),z{ii}(2),z{ii}(3),['   ',num2str(ii)])
  end
end

end

%% Sub-functions

function o = arg3d(a, b)
   o = atan2(norm(cross(b,a)), dot(b,a));
end

%DO NOT USE, works only for XY-plane
% function o = arg(w)
%    o = atan2(w(2),w(1));
% end

function [rho,sigma] = velocity_parameters(theta,phi)
% From "Smooth, easy to compute interpolating splines" by John D. Hobby
% <http://www.springerlink.com/content/p4w1k8w738035w80/>

a = 1.597;
b = 0.07;
c = 0.37;

st = sin(theta);
ct = cos(theta);
sp = sin(phi);
cp = cos(phi);

alpha = a*(st-b*sp)*(sp-b*st)*(ct-cp);
rho   = (2+alpha)/(1+(1-c)*ct+c*cp);
sigma = (2-alpha)/(1+(1-c)*cp+c*ct);

end

function [Q, coeffs] = plot_bezier(P1,P2,P3,P4,N,plotcurve,linestyle)

t = linspace(0,1,N)';

c1 = 3*(P2 - P1);
c2 = 3*(P1 - 2*P2 + P3);
c3 = -P1 + 3*(P2-P3) + P4;

Q = t.^3*c3 + t.^2*c2 + t*c1 + repmat(P1,[N 1]);

coeffs = [c3', c2', c1', P1'];

if plotcurve
    plot3(Q(:,1),Q(:,2),Q(:,3),linestyle{:});
end

end