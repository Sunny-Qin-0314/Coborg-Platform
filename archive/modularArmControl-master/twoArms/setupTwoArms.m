addpath(genpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi-matlab-1.2-rev2085'));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\hebi API\matlab_SEA'));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\manipulatorChain\backpackArmControl'));

sendCommands = 1;
animate =~sendCommands;

aRight = [pi/2; -pi];
lengthsRight = [.24; .2];
aLeft = [-pi/2; -pi];
lengthsLeft = lengthsRight;
rb = [0;0;0];

% setDefaultGains_XSeries( 'TrayArm')
n = length(aRight);


%% Create animation
if animate
    links = {};
    for i = 1:n-1
        links = [links, {{'X5-9'},...
            {'X5Link','ext',lengthsRight(i),'twist',aRight(i+1)}  }];
    end
    links = [links, {{'X5-9'},...
        {'X5Link','ext',lengthsRight(n),'twist',0}  }];
    pltRight = HebiPlotter('JointTypes', links,'resolution','low');
    pltRight.setBaseFrame( [R_x(aRight(1) ), rb; 0 0 0 1] );
    pltRight.plot(zeros(1,n))

    links = {};
    for i = 1:n-1
        links = [links, {{'X5-9'},...
            {'X5Link','ext',lengthsLeft(i),'twist',aLeft(i+1)}  }];
    end
    links = [links, {{'X5-9'},...
        {'X5Link','ext',lengthsLeft(n),'twist',0}  }];
    pltLeft = HebiPlotter('JointTypes', links,'resolution','low');
    pltLeft.setBaseFrame( [R_x(aLeft(1) ), rb; 0 0 0 1] );
    pltLeft.plot(zeros(1,n))
end



%% create kinematics object
kinRight = HebiKinematics();
for i =  1:n-1
    kinRight.addBody('X5-4', 'PosLim', [-pi pi]);
    kinRight.addBody('X5Link', 'ext', lengthsRight(i), 'twist', aRight(i+1));
end
 kinRight.addBody('X5-4', 'PosLim', [-pi pi]);
kinRight.addBody('X5Link', 'ext', lengthsRight(n), 'twist', 0);
kinRight.setBaseFrame( [R_x(aRight(1) ), rb; 0 0 0 1] );
trajGenRight = HebiTrajectoryGenerator(kinRight);


%% create kinematics object
kinLeft = HebiKinematics();
for i =  1:n-1
    kinLeft.addBody('X5-4', 'PosLim', [-pi pi]);
    kinLeft.addBody('X5Link', 'ext', lengthsLeft(i), 'twist', aLeft(i+1));
end
 kinLeft.addBody('X5-4', 'PosLim', [-pi pi]);
kinLeft.addBody('X5Link', 'ext', lengthsLeft(n), 'twist', 0);
kinLeft.setBaseFrame( [R_x(aLeft(1) ), rb; 0 0 0 1] );
trajGenLeft = HebiTrajectoryGenerator(kinRight);


% make robot group
robotRight = HebiLookup.newGroupFromNames('*', {'X-00041', 'X-00037'}); % right arm
cmdRight = CommandStruct();
cmdRight.position = NaN(1,n);
cmdRight.velocity = NaN(1,n);
cmdRight.effort = NaN(1,n);
robotRight.set(cmdRight);

robotLeft = HebiLookup.newGroupFromNames('*', {'X-00034', 'X-00025'}); % left arm
cmdLeft = CommandStruct();
cmdLeft.position = NaN(1,n);
cmdLeft.velocity = NaN(1,n);
cmdLeft.effort = NaN(1,n);
robotLeft.set(cmdLeft);

setTwoArmGains;