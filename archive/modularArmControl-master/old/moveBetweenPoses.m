% this script moves the arm between various poses that were loaded from a
% file.

% Flags:
sendCommands = 1;
animate =~sendCommands;

% paths:
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi9-20-2017');

addpath(genpath(pwd));
addpath(genpath('C:\Users\medgroup01\Documents\Julian\hebi API\matlab_SEA'));

% load armData.mat;
% close all;
% % lengths = [ 0.0250 +  0.0245*11.5 ,  0.0250 +  0.0245*8 , 0.0250/2 +  0.0245* 7.75]
% lengths = [ 0.0250 +  0.0245*11.5 ,  0.0250 +  0.0245*8 , 0.0250/2 + .01 ]
% a = [ 0, -1.6247    0.0182      ].'
% th = [0, -pi/4, pi/4; 0, -pi/4, pi/4; 0, -pi/4, pi/4];
S= load('overheadConstrainedD_3.mat');
a = S.a;
th = S.th;
rb = S.rb;
lengths = S.lengths;
% conditioning angles
th = mod(th, 2*pi);
th(th>pi) = th(th>pi) - 2*pi;
a = mod(a, 2*pi);
a(a>pi) = a(a>pi) - 2*pi;

n = length(a);

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
    %     plt.setBaseFrame( [R_x(a(1) + pi/2), rb; 0 0 0 1] );
    plt.setBaseFrame( [R_x(a(1) ), rb; 0 0 0 1] );
    plt.plot(th(:,1))
end

%% create kinematics object
kin = HebiKinematics();
for i =  1:n-1
    kin.addBody('X5-4', 'PosLim', [-pi pi]);
    kin.addBody('X5Link', 'ext', lengths(i), 'twist', a(i+1));
end
kin.addBody('X5-4', 'PosLim', [-pi pi]);
kin.addBody('X5Link', 'ext', lengths(n), 'twist', 0);
% kin.setBaseFrame( [R_x(a(1) + pi/2), rb; 0 0 0 1] );
kin.setBaseFrame( [R_x(a(1) ), rb; 0 0 0 1] );
trajGen = HebiTrajectoryGenerator(kin);


%% create arm group, gains and command structures
if sendCommands
    makeRobotGroup;
    setGainsBackpack;
end

%% start joystick
joy = vrjoystick(1);
[axes,buttons,povs] = read(joy);


if ~sendCommands
    lastPos = zeros(1,n);
else
    
    try
        fbk = robot.getNextFeedback();
    catch err
        disp(err.message)
    end
    lastPos =fbk.position;
end


disp(['Mean voltage: ' num2str(mean(fbk.voltage), '%2.1f'), ' V']);


numGoals = size(th, 2);
goalNum = 0;
lastGoalNum = goalNum;
running = 1;
mode = 'gravComp';

while running
    
    if mean(fbk.voltage)<30
        disp(['Low voltage: ' num2str(mean(fbk.voltage), 1), ' V']);
    end
    
    % read the joystick
    [axes,buttons,povs] = read(joy);
    % to do: add alternate input from voice command
    % - add a flag to tell it to ignore voice input
    
    if any(buttons(1:numGoals))
        newGoalNum = find(buttons(1:numGoals));
        goalNum = newGoalNum(1); % only take the lowest number pressed
        mode = 'goalPoses';
    end
    running = ~any(buttons(9:10));
    
    if ((goalNum~=lastGoalNum)&&running&&(strcmp(mode, 'goalPoses') ))
        lastGoalNum = goalNum;
        disp(['go to goal: ',num2str(goalNum)]);
        positionEnd = th(:,goalNum).';
        if sendCommands
            try
                fbk = robot.getNextFeedback();
            catch err
                disp(err.message)
            end
            lastPos =fbk.position;
        end
        moveArmToTheta;
%         moveToThetaBumpStop; % not working well yet
    end
    
    if buttons(12) % if button 12 is pressed, go into grav comp
        mode = 'gravComp';
    end
    
    if strcmp(mode,'gravComp')
        disp('gravity comp mode')
        lastGoalNum = 0;
        gravityCompMode;
    end
    
    if strcmp(mode, 'holdPose')
        % this mode will hold lastPos until a different button pressed.
        if sendCommands
            if sendCommands
                try
                    fbk = robot.getNextFeedback();
                catch err
                    disp(err.message)
                end
            end
            
            % compensate for gravity
            gravityVec = [fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
            % dealing with non identity base frames
            baseFrame = kin.getBaseFrame();
            gravityVec = baseFrame(1:3,1:3)' * gravityVec;
            gravCompEfforts = kin.getGravCompEfforts(fbk.position, gravityVec);
            % send to hardware
            cmd.position = lastPos;
            cmd.velocity = nan(1,n);
            cmd.effort = gravCompEfforts;
            try
                robot.set(cmd);
            catch err
                disp(err.message);
            end
        end
        if buttons(numGoals+1)
            numGoals = numGoals+1;
            th = [th, lastPos.'];
            disp(['added goal: ',num2str(numGoals)]);
            goalNum = numGoals;
            lastGoalNum = goalNum;
            
        end
        pause(.01);
    end % endif strcmp(mode, 'holdPose')
    
    % d-pad will make it push in a direction
    if (povs~=-1)
        mode = 'push';
        % right now, push up
        % uses cartesian admittance control to apply linear force
        pos = lastPos;
        switch povs
            case 0
                directionToPush = [0;0;1]; % this will be relative to base frame, not gravity
                pushString = 'up';
            case 180
                directionToPush = [0;0;-1];
                pushString = 'down';
            case 90
                directionToPush = [1;0;0];
                pushString = 'forward';
        end
    end
    
    if strcmp(mode,'push')
        disp([mode ' ' pushString]);
        lastGoalNum = 0;
        pushImpedanceMode;
        %                 pushAdmittanceMode;
        % push with pure torque control
        %       pushTorqueMode;
    end
    
    if animate
    drawnow;
    end
end

if sendCommands
    setToNaN;
end

