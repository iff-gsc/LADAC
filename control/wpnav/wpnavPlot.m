function [] = wpnavPlot( waypoints, wp_radius, varargin )

dim = 2;
for i = 1:length(varargin)
    if strcmp(varargin{i},'dim')
        dim = varargin{i+1};
    end
end

if dim == 2
    plot(waypoints(1,:),waypoints(2,:),'k--o')
elseif dim == 3
    plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'k--o')
    view(-37.5,30)
end
hold on

res = 100;
ang = linspace(0,2*pi,res);
num_wp = size(waypoints,2);
for i = 1:num_wp
    if i>1 && i<num_wp
        
        circ_seg = wpnavCircSeg( waypoints(:,i-1:i+1), wp_radius );
        
        alpha_s = linspace(0,circ_seg.angle,res);
        circ = zeros(3,res);
        for j = 1:res
            circ(:,j) = circ_seg.center + axisAngle(circ_seg.start-circ_seg.center,circ_seg.n,alpha_s(j));
        end

        if dim == 2
            plot(circ(1,:),circ(2,:),'r-')
        elseif dim == 3
            plot3(circ(1,:),circ(2,:),circ(3,:),'r-')
        end
        wp_rad = circ_seg.wp_rad;
    else
        wp_rad = wp_radius;
    end
    
    if dim == 2
        x = wp_rad*cos(ang);
        y = wp_rad*sin(ang);
        plot(waypoints(1,i)+x,waypoints(2,i)+y,'b-')
    end

end

grid on
box on
axis equal

xlabel('East, m')
ylabel('North, m')

if dim == 3
    zlabel('Altitude, m')
end

hold off

end
