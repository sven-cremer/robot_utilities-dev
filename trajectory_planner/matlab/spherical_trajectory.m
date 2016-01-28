function [points] = spherical_trajectory(max_theta,max_azumith,radius,resolution,axis,spherical)
% spherical_trajectory  Generates a circular trajectory in 2D and 3D.
%   generates a set of points that represent a circular trajectory
%   max_theta is the maximum theta value the trajectory can rotate around.
%       [0,2pi]
%   max_azumith is the maximum azumith value the trajectory can rotate the
%       3rd axis
%   radius is the radial distance can traverse form the origin to the
%       circumferance
%   resolution is the number of points on the trajectory
%   axis is a flag that determine in what axis the circle will be
%   located on. 0 means the circle will be on the x and y axis. 1 means the
%   circle will be on the x and z axis. 2 means the circle will be on the y
%   and z axis.
%   spherical is a flag that determines is the circluar trajectory extends
%   into 3D space. Typically the resulting trajectory represents something
%   similar to a cylinder.

%%
close all
%radius = 0.2;
%resolution = 100;
%max_theta = 2*pi;
%max_azumith = pi;
%sperical = 0
%axis = 1
theta = max_theta;  %Max value of 2pi
azumith = max_azumith; %Max value of pi
points = zeros(resolution,3);
figure(1);
hold on
for i=1:resolution
    %By default, circle is on x and y axis
    x = radius*cos(theta*(i/resolution));
    y = radius*sin(theta*(i/resolution));
    z = 0;
    %Circle on x and z axis
    if(axis == 1)
        x = radius*sin(theta*(i/resolution));
        y = 0;
        z = radius*cos(theta*(i/resolution));
        %Circle on y and z axis
    elseif(axis == 2)
        x = 0
        y = radius*sin(theta*(i/resolution));
        z = radius*cos(theta*(i/resolution));
    end
    if(spherical == 1)
        if(y == 0)
            x = x*sin(azumith*(i/resolution));
            y = radius*cos(azumith*(i/resolution));
            z = z*sin(azumith*(i/resolution));
        elseif(x == 0)
            x = radius*cos(azumith*(i/resolution));
            y = y*sin(azumith*(i/resolution));
            z = z*sin(azumith*(i/resolution));
        elseif(z == 0)
            x = x*sin(azumith*(i/resolution));
            y = y*sin(azumith*(i/resolution));
            z = radius*cos(azumith*(i/resolution));
        end
    end
    %plot3(x,y,z,'*');
    points(i,:) = [x,y,z];
end
%xlabel('x');
%ylabel('y');
%zlabel('z');
end

function plot_trajectory(points)
for i=1:size(points(:,1))
    plot3(points(i,1),points(i,2),points(i,3),'*');
end
end
