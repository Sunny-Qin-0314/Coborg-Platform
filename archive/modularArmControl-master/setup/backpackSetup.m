
robot = {'BaseModuleZRY', 'LinkModule', ...
    'RotJointModule',  'LinkModule',...
    'RotJointModule',  'LinkModule',...
    'RotJointModule', 'EndEffectorVarLXModule'};
% links are [length, twist]

%	     rb = [0.4000         0    0.0065]
% lengths =    0.2317    0.2048    0.2559

dvars = [0.0065 ;... %BaseModuleZ
    0.4000 ;...%LinkModule length
    1.3343 - pi/2;...%LinkModule twist
    0.2317	 ;...%LinkModule length
    2.1630  ;...%LinkModule twist
    0.2048	;... %LinkModule length
    -3.7736 ;...%LinkModule twist
    0.2559 ;...%EndEffectorVarLModule
    ];

theta = [...
    1.2512    1.0225    1.0734;...
    -0.7863   -1.4381   -0.7035;...
    -0.8820    4.9165    5.4988];

theta = [theta, [   -2.4394  ; -2.4953 ;  -2.5588]]; % add folded pose

DVars = [1; 2; 0; 2; 0; 2; 0; 1];

n = 3;

% % for testing:
% close all;
% nModules = length(robot);
% robotDVarsToHebiKin
% fk = kin.getFK('Output', [0,0,0]);
% figure; xyz = squeeze(fk(1:3,4,:));
% plot3(xyz(1,:), xyz(2,:), xyz(3,:));
% hold on;
% % quiver3(xyz(1,end), xyz(2,end), xyz(3,end), fk(1,1,end), fk(2,1,end), fk(3,1,end), .5)
% xlabel('x'); ylabel('y'); zlabel('z')
% axis([-1 1 -1 1 -1 1]*1.5)
% for i = 1:3
% fk = kin.getFK('Output', theta(:,i));
%  xyz = squeeze(fk(1:3,4,:));
% plot3(xyz(1,:), xyz(2,:), xyz(3,:));
% end
