% IK test

% Flags:
sendCommands = 1;
animate = 1;
S = load('overheadConstrainedD_3.mat');
a = S.a;
lengths = S.lengths;
th = S.th;
rb = S.rb;
close all;

% paths:
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi9-20-2017');
addpath(genpath(pwd));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\hebi API\matlab_SEA'));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\manipulatorChain\backpackArmControl'));

addpath('C:\Users\TrashManUser\Documents\MATLAB\hebi');
addpath(genpath('C:\Users\TrashManUser\Documents\MATLAB\matlab_SEA'));
addpath('C:\Users\TrashManUser\Documents\MATLAB\manipulatorOptimization\manipOptRealArm\geometryTools');
addpath(genpath('C:\Users\TrashManUser\Documents\MATLAB\manipulatorOptimization\backpackArmControl'));

n = length(a);
thInit= th(:,1);

%% Create animation
if animate
    links = {};
    for i = 1:n-1
        links = [links, {{'X5-9'},...
            {'X5Link','ext',lengths(i),'twist',a(i+1)}  }];
    end
    links = [links, {{'X5-9'},...
        {'X5Link','ext',lengths(n),'twist',0}  }];
    plt = HebiPlotter('JointTypes', links,'resolution','low');
    plt.setBaseFrame( [R_x(a(1)), rb; 0 0 0 1] );
    plt.plot(thInit);
end

%% create kinematics object
kin = HebiKinematics();
for i =  1:n-1
    %     kin.addBody('X5-4', 'PosLim', [0 2*pi]);
    kin.addBody('X5-4', 'PosLim', [-pi pi]);
    kin.addBody('X5Link', 'ext', lengths(i), 'twist', a(i+1));
end
% kin.addBody('X5-4', 'PosLim', [0 2*pi]);
kin.addBody('X5-4', 'PosLim', [-pi pi]);
kin.addBody('X5Link', 'ext', lengths(n), 'twist', 0);
kin.setBaseFrame( [R_x(a(1)), rb; 0 0 0 1] );
% trajGen = HebiTrajectoryGenerator(kin);

joy = vrjoystick(1);
[axes,buttons,povs] = read(joy);

%% create arm group, gains and command structures
if sendCommands
    makeRobotGroup;
    setGainsBackpack;
end

fk = kin.getFK('EndEffector', thInit);
xyz = fk(1:3,4);
thLast = thInit;
running = 1;
hold on;
outScatter = scatter3( xyz(1), xyz(2), xyz(3), 'filled');
hold off;

while running
    [axes,buttons,povs] = read(joy);
    running = ~any(buttons(9:12));
    
    axes(abs(axes)<.1) = 0;
    
    xyz = xyz + [-axes(2); axes(1); -axes(4)] * .01;
    
    thNow = kin.getInverseKinematics('Xyz', xyz, 'InitialPositions', thLast);
    thLast = thNow;
    
    if sendCommands
        cmd.position = thNow;
        robot.set(cmd);
    end
    
    if animate
        plt.plot(thNow);
            set(outScatter, 'xdata', xyz(1), 'ydata', xyz(2), 'zdata', xyz(3));
    else
        pause(.01);
    end
end

if sendCommands
    setToNaN;
end