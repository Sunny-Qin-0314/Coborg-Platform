
thMin = [-pi/4, -pi/6 ,   -pi/6, -pi, -inf ];
thMax = [pi/4,  pi/2+pi/6 , pi  , pi, inf ]; 

kin = HebiKinematics();
kin.addBody('X8-9', 'PosLim', [thMin(1) thMax(1)]);
% kin.addBody('GenericLink','com',[.1 .1 .1],...
%             'out',([eye(4,3),[-.022 0 .047 1]']*(rotz(pi/2)*rotx(pi/2))),...
%             'mass',0.1);
kin.addBody('X5-HeavyBracket', 'mount', 'right-inside'); % 90-deg bracket
kin.addBody('X8-16', 'PosLim', [thMin(2) thMax(2)]);
% kin.addBody('GenericLink','com',[0 0 0],'out',rotz(pi/2),'mass',0);
kin.addBody('X5Link', 'ext', 0.525, 'twist', pi); % 500mm links -> .525
kin.addBody('X8-16', 'posLim', [thMin(3) thMax(3)]); % Restrict for IK to avoid elbow flip
% kin.addBody('X5-9', 'posLim', [-pi/2 pi]);
kin.addBody('X5Link', 'ext', 0.525, 'twist', pi);
kin.addBody('X5-9','posLim', [thMin(4) thMax(4)]);
kin.addBody('X5-LightBracket', 'mount', 'right'); % 90-deg bracket
kin.addBody('X5-9');

% gripper End Effector 
screwdriverOut = [
    1 0 0 0
    0 1 0 0
    0 0 1 0.058
    0 0 0 1 ];

kin.addBody( 'GenericLink', ...
    'com', [0 0 0.02], ...
    'out', screwdriverOut, ...
    'mass', 0.2 );

%print configuration information to screen (uncheck when you want to see)
% kin.getBodyInfo();
% kin.getJointInfo();

% newFrame = eye(4)*rotz(-pi/2);
newFrame = eye(4)*roty(pi);
kin.setBaseFrame(newFrame);


names = {'X-80263', 'X-80312', 'X-80323', 'X-00547',  'X-00399' };
group = HebiLookup.newGroupFromNames('*', names);
cmd = CommandStruct();
n = group.getNumModules;


% set up claw group
clawModuleName = {'X-00529'};
claw = HebiLookup.newGroupFromNames('*', clawModuleName);
cmdClaw = CommandStruct();
cmdClaw.position = nan;
cmdClaw.velocity = nan;
cmdClaw.effort = nan;


setGainsBeltArm;
