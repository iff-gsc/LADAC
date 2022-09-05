function [traj] = hobbysplines_Simulink(points,tension)


%% Parse inputs
cycle  = false;
%bezierpoints = 50;

if cycle
  points{end+1} = points{1};
end

Npoints = numel(points);

%z = cell(Npoints, 1); % points
%w = cell(Npoints, 1); % unit vectors of direction of curve through each point
%tin = cell(Npoints, 1); % tension of curve in to point
%tout = cell(Npoints, 1); % tension of curve out from point

z = {zeros(1,3,'single');zeros(1,3,'single')};
w = {zeros(1,3,'single');zeros(1,3,'single')};
tin  = {zeros(1,1,'single');zeros(1,1,'single')};
tout = {zeros(1,1,'single');zeros(1,1,'single')};

for n = 1:Npoints
  
  pp = points{n};
  
  w{n} = single([NaN NaN NaN]);
  tout{n} = single(tension);
  tin{n} = single(tension);

  z{n} = pp{1};
  w{n} = pp{2};
      
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

%q = nan(bezierpoints*(Npoints-1),3);

max_points = 1;
degree = 5;
traj = trajInitSingle(max_points, degree);
traj.num_sections_set = single(Npoints-1);

% traj.num_sections_max = Npoints;
% traj.active_section = 0;
% traj.current_time = 0;

for ii = 1:Npoints-1

  theta = arg3d(w{ii}, z{ii+1}-z{ii});
  phi   = arg3d(z{ii+1}-z{ii}, w{ii+1});
  
  [rho,sigma] = velocity_parameters(theta,phi);
  
  P1 = z{ii};
  P2 = z{ii}+rho/(3*tout{ii})*norm(z{ii+1}-z{ii})*w{ii};
  P3 = z{ii+1}-sigma/(3*tin{ii+1})*norm(z{ii+1}-z{ii})*w{ii+1};
  P4 = z{ii+1};
  
  [coeffs] = calc_bezier_coeffs(P1, P2, P3, P4);

 traj.sections(ii).pos_x = [0, 0, coeffs(1,:)];
 traj.sections(ii).pos_y = [0, 0, coeffs(2,:)];
 traj.sections(ii).pos_z = [0, 0, coeffs(3,:)];

%  traj.sections(ii).t = 0;
%  traj.sections(ii).vel = 0;
%  traj.sections(ii).distance = 0;
%  traj.sections(ii).arc_length = 0;
 
end

traj = trajSetArcLength(traj);


end

%% Sub-functions

function o = arg3d(a, b)
   o = atan2(norm(cross(b,a)), dot(b,a));
end


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

function [coeffs] = calc_bezier_coeffs(P1,P2,P3,P4)

c1 = 3*(P2 - P1);
c2 = 3*(P1 - 2*P2 + P3);
c3 = -P1 + 3*(P2-P3) + P4;

coeffs = [c3', c2', c1', P1'];

end

function traj = trajInitSingle( num_sections_max, degree )

traj = struct( ...
    'num_sections_max', single(num_sections_max), ...
    'num_sections_set', single(0), ...
    'sections', repmat( trajSectionInitSingle(degree), num_sections_max, 1 ), ...
    'active_section', single(0), ...
    'current_time', single(0), ...
    'arc_length', single(0), ...
    'distance', single(0),...
    'is_repeated_course', false, ...
    'polynomial_degree', single(degree) ...
    );

end

function traj_section = trajSectionInitSingle(degree)

traj_section = struct( ...
    'pos_x', zeros(1, degree+1, 'single'), ...
    'pos_y', zeros(1, degree+1, 'single'), ...
    'pos_z', zeros(1, degree+1, 'single'), ...
    'vel',   zeros(1, degree+1, 'single'), ...
    't', single(0), ...
    'arc_length', single(0),...
    'distance', single(0), ...
    'polynomial_degree', single(degree) ...
    );

end