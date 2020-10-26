%% setup
% add paths
addpath(genpath(pwd));
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi1-2-2019');

% %% move other arms out of the way
% pushArm = HebiLookup.newGroupFromNames('*', {'base_temp', 'X-00009', 'elbow_temp'});
% backpackAngles = [-2.4394, -2.4953, -2.5588];
% pushArmCmd= CommandStruct();
% pushArm.setCommandLifetime(0); % so the commands don't expire
% pushArmCmd.position = backpackAngles;
% pushArm.set(pushArmCmd)


%% create arm object
setupBeltArm;
elbowBend = pi/2 - pi/12;
homePos = [0, elbowBend,   2*elbowBend,   elbowBend+pi/2, pi/2];

% joystick object:
joy = vrjoystick(1);
trajGen = HebiTrajectoryGenerator(kin);

fkHome = kin.getForwardKinematics('endEffector',homePos);
    
xmax = [-.05; .3; .25; pi; pi; pi];
xmin = [-.75; -.3; -1; -pi; -pi; -pi];
dt = .01;

%% misc vars
gravity = [0; 0; -9.81];
axesThres  = .2;

EA = -SpinCalc('DCMtoEA123', fkHome(1:3,1:3), 1e-6, 0) * pi/180; % converts to RPY
EA(EA>pi) = EA(EA>pi) - 2*pi;
EA(EA<-pi) = EA(EA<-pi) + 2*pi;
x0 = [fkHome(1:3,4);  EA.'];


positions = [fbk.position; ...
    homePos];
lastPosition = homePos;
trajectory = trajGen.newJointMove(positions);
trajGen.setSpeedFactor(1);
running = 1;
moveToInitialPos; % Moves the arm!


% running = 1;
% while running % hold loop
%   % get joystick buttons and map joystick to coordinates
%     [axes,buttons,povs] = read(joy);
%     
%     axes(abs(axes)<axesThres) = 0;
%     running = ~any(buttons(9:12));
% cmd.effort = nan(1,5);
% group.set(cmd);
% pause(.001);
% end



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

    % joystick commands: move x0
    x0(1) = x0(1) + -axes(2)*.005; % x
    x0(2) = x0(2) +  axes(1)*.001; % y
    x0(3) = x0(3) + -axes(4)*.005; % z
%     x0(4) = x0(4) + (buttons(7)-buttons(8))*.01; % roll   
    x0 = max(x0, xmin);
    x0 = min(x0, xmax);
    
    
    % get fk 
    fk = kin.getForwardKinematics('Output',fbk.position);
    
    % get jacobians
%     J = kin.getJacobian('endEffector',fbk.position);
    
    % F = J* Tau.'
%     F = J*cmd.effort;

         
    % convert to commands for arm
    tipAx =  fk(1:3,1:3,1)* [1;0;0];
    position = kin.getInverseKinematics('xyz',x0(1:3),...
        'TipAxis', tipAx,...
     'InitialPositions',fbk.position,...
            'MaxIterations',1000);
    
    disp(num2str(round([x0(1:3); tipAx],2).'))

        
    % gravity compensation efforts
%     efforts = kin.getGravCompEfforts(fbk.position, gravity);

    position = min(position, thMax);
    position = max(position, thMin);
    
    cmd.position = position;
    cmd.velocity = nan(1,n);
    cmd.effort = nan(1,n);
%     cmd.effort = efforts;
    
    group.set(cmd);
    claw.set(cmdClaw);
    
    pause(dt);
    
end

cmd.position = nan(1,n);
cmd.velocity = nan(1,n);
cmd.effort = nan(1,n);
group.set(cmd);

