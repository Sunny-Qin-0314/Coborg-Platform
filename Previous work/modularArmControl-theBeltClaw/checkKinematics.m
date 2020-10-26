
% fk = kin.getForwardKinematics('OutputFrame',zeros(1,5));
close all;
figure;
h = plot3(0,0,0, 'k');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

while true
fk = kin.getForwardKinematics('OutputFrame',homePos);
fbk = group.getNextFeedback();
% fk = kin.getForwardKinematics('OutputFrame',fbk.position);
xyz = squeeze(fk(1:3,4,:));
set(h, 'xdata', xyz(1,:), 'ydata', xyz(2,:), 'zdata', xyz(3,:));
axis equal
pause(.01);
end