%% setup
% add paths
addpath(genpath(pwd));
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi1-2-2019');


%% create arm object
setupBeltArm;
elbowBend = pi/2 - pi/12;
homePos = [0, elbowBend,   2*elbowBend,   elbowBend+pi/2, pi/2];
position = homePos;
% joystick object:
joy = vrjoystick(1);

dt = .01;

%% misc vars
gravity = [0; 0; -9.81];
axesThres  = .2;
running = 1;



%% main loop
while running
    % get feedback
    fbk = group.getNextFeedback();
    
    % get joystick buttons and map joystick to coordinates
    [axes,buttons,povs] = read(joy);
    
    axes(abs(axes)<axesThres) = 0;
    running = ~any(buttons(9:12));
    
    % use joystick commands: claw
    % open/close claw
    if buttons(5) % close
        cmdClaw.effort = -6; % change to good torque close
        cmdClaw.position = nan;
    elseif buttons(6) % open
        cmdClaw.effort = nan;
        cmdClaw.position = -.5; % change to good position for open
    end

   
    
    % get fk 
    fk = kin.getForwardKinematics('Output',fbk.position);
    
        
    % gravity compensation efforts
%     efforts = kin.getGravCompEfforts(fbk.position, gravity);

%     position = min(position, thMax);
%     position = max(position, thMin);
    
    cmd.position = position;
    cmd.velocity = nan(1,n);
%     cmd.effort = efforts;
    cmd.effort = nan(1,n);
    
    group.set(cmd);
    claw.set(cmdClaw);
    
    pause(dt);
    
end

cmd.position = nan(1,n);
cmd.velocity = nan(1,n);
cmd.effort = nan(1,n);
group.set(cmd);

