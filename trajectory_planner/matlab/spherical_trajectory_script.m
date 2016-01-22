
points = spherical_trajectory(2*pi,pi,0.2,100,0,0);
figure(1);
for i=1:size(points(:,1))
    plot3(points(i,1),points(i,2),points(i,3),'*');
end

% points = spherical_trajectory(2*pi,pi,0.2,100,1,0);
% figure(2);
% for i=1:size(points(:,1))
%     plot3(points(i,1),points(i,2),points(i,3),'*');
% end
% 
% points = spherical_trajectory(2*pi,pi,0.2,100,2,0);
% figure(3);
% for i=1:size(points(:,1))
%     plot3(points(i,1),points(i,2),points(i,3),'*');
% end
% 
% points = spherical_trajectory(2*pi,pi,0.2,100,0,1);
% figure(4);
% for i=1:size(points(:,1))
%     plot3(points(i,1),points(i,2),points(i,3),'*');
% end
% 
% points = spherical_trajectory(2*pi,pi,0.2,100,1,1);
% figure(5);
% for i=1:size(points(:,1))
%     plot3(points(i,1),points(i,2),points(i,3),'*');
% end
% 
% points = spherical_trajectory(2*pi,pi,0.2,100,2,1);
% figure(6);
% for i=1:size(points(:,1))
%     plot3(points(i,1),points(i,2),points(i,3),'*');
% end