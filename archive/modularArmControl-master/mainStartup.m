% Julian Whitman
% main file for controlling the backpack arm in multiple modes


% Flags:
sendCommands = 1; % make a real robot or not?
animate =~sendCommands;
withGripper =0; % use the gripper
% useVoiceInput = 0; % leave this zero...


% paths:
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi6-20-2018');
addpath(genpath(pwd));

% load file with robot design in it
% close all;
% robot = p.robot;
% DVars = p.DVars;
% n = p.nJoints;

backpackSetup;


% conditioning angles
th = mod(theta, 2*pi);
th(th>pi) = th(th>pi) - 2*pi;


%% hand tuned max and min angles
thmax = 2*pi*[1,1,1];
thmin = -2*pi*[1, 1, 1];



%% Create animation
if animate
    figure;
    fkPlot = plot3(0,0,0);
end

%% create kinematics object
addpath('C:\Users\medgroup01\Documents\Julian\manipulatorChain\manipulatorOptimization2\postprocessing');
nModules = length(robot);
robotDVarsToHebiKin;
trajGen = HebiTrajectoryGenerator(kin);


%% create arm group, gains and command structures
if sendCommands
    if n==4
        makeRobotGroup4;
    elseif n == 3
        makeRobotGroup;
    elseif n ==5
        makeFetchArmGroup
    end
    %     setDefaultGains_XSeries;
    %             setGainsBackpack;
    setGainsRover;
    
end

if withGripper
    setupGripper;
end

%% start joystick
joy = vrjoystick(1);
[axes,buttons,povs] = read(joy);
nButtons = length(buttons); % not all joysticks have same number of buttons!


if ~sendCommands
    lastPos = zeros(1,n);
    pos = lastPos;
else
    try
        fbk = robot.getNextFeedback();
    catch err
        disp(err.message)
    end
    lastPos =fbk.position;
    disp(['Mean voltage: ' num2str(mean(fbk.voltage), '%2.1f'), ' V']); 
end



displayInstructions;


numGoals = size(th, 2);
if numGoals>6
    numGoals = 6;
end
goalNum = 0;
lastGoalNum = NaN;
running = 1;
mode = 'gravComp';

while running
    
    % grab most recent fbk
    if sendCommands
        try
            fbk = robot.getNextFeedback();
        catch err
            disp(err.message)
        end
        % check if voltage is too low
        if mean(fbk.voltage)<24
            disp(['Low voltage: ' num2str(mean(fbk.voltage), 1), ' V']);
        end
    end
    
    
    
    % Look to see if there is a new mode to go to:
    checkForNewMode;
    
    % move to a new goal pose
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
    
    % go to grav. comp
    if strcmp(mode,'gravComp')
        disp('gravity comp mode')
        gravityCompMode;
    end
    
    % hold the pose its at
    if strcmp(mode, 'holdPose')
        % this mode will hold lastPos until a different button pressed.
        holdPoseMode;
    end % endif strcmp(mode, 'holdPose')
    
    % push in a direction
    if strcmp(mode,'push')
        disp([mode ' ' pushString]);
        pushImpedanceMode;
    end
    
    % follow with IK
    if strcmp(mode,'follow')
        disp('Following mode');
        %         followHandMode;
        followMode;
    end
    
    if animate
        drawnow;
    end
end

if sendCommands
    setToNaN;
end

