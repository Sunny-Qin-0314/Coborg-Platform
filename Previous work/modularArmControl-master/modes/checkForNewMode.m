% this script checks for new modes, and updates the mode number

% check for voice input
newestWord = checkForNewWord;

useVoiceInput = 0;
if useVoiceInput % if we have voice input, use instead of joystick
axes = zeros(1,4);
buttons = zeros(1,12);
povs = -1;
    switch newestWord
        case 'hold'
            buttons(11) = 1;
        case 'free'
            buttons(12) = 1;
        case 'pose one'
            buttons(1) = 1;
        case 'pose two'
            buttons(2) = 1;
        case 'pose three'
            buttons(3) = 1;
        case 'pose four'
            buttons(4) = 1;
        case 'quit'
            buttons(10) = 1;
        case ''
            % do nothing
        otherwise
            disp(['word not recognized: ' newestWord]);
    end
else
    % read the joystick
    [axes,buttons,povs] = read(joy);
end

% check if still running
% running = ~any(buttons(9:10));
running = ~any(buttons(end-3 : end-2));

if ~running
    remainInMode = 0;
end

% check if switching to goal poses
if any(buttons(1:numGoals))
    newGoalNum = find(buttons(1:numGoals));
    if newGoalNum ~= lastGoalNum
        goalNum = newGoalNum(1); % only take the lowest number pressed
        mode = 'goalPoses';
        remainInMode = 0;
    end
end


% check if switching to grav comp
if buttons(end) % if button 12 is pressed, go into grav comp
    mode = 'gravComp';
    disp('grav Comp mode')
    remainInMode = 0;
    goalNum = 0;
    lastGoalNum =0;
end

% check if switching to hold pose
if buttons(end-1)
    remainInMode = 0;
    mode = 'holdPose';
    disp('Hold pose mode');
    goalNum = 0;
    lastGoalNum =0;
    lastPos = fbk.position;
end

% check if switching to push mode
% d-pad will make it push in a direction
if (povs~=-1)
    mode = 'push';
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
        otherwise
            directionToPush = [0;0;1]; % this will be relative to base frame, not gravity
            pushString = 'up';
            % TO DO: make pushes go in diag directions
    end
    goalNum = 0;
    lastGoalNum =0;
    remainInMode = 0;
    
end

% check to see if entering IK mode
if buttons(end-4)&&buttons(end-5)
    mode = 'follow';
    remainInMode = 0;
    goalNum = 0;
    lastGoalNum =0;
end

if (abs(axes(3))>.05)&&(withGripper)
    cmdGripper.position = cmdGripper.position + .05*axes(3);
    cmdGripper.position = max(cmdGripper.position, gripperMin);
    cmdGripper.position = min(cmdGripper.position, gripperMax);
    gripper.set(cmdGripper);
    
end
