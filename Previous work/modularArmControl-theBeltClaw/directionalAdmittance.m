%% Julian Whitman
% 1-9-2019
% The Belt claw with directional Admittance
% TODO: when the claw is closed, try adding some lifting force so that the user will not feel the weight of the arm. 

%% setup
% add paths
addpath(genpath(pwd));
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi1-2-2019');

%% flags
directionalAdmittanceOn = 1;
plottingLog = 0;
plottingArm = 0;
alpha = .1; % low pass on output position

%% move other arms out of the way
pushArm = HebiLookup.newGroupFromNames('*', {'base_temp', 'X-00009', 'elbow_temp'});
backpackAngles = [-2.4394, -2.4953, -2.5588];
pushArmCmd= CommandStruct();
pushArm.setCommandLifetime(0); % so the commands don't expire
pushArmCmd.position = backpackAngles;
pushArm.set(pushArmCmd)


%% create arm object
setupBeltArm;
elbowBend = pi/2 - 12*pi/181;
homePos = [0, elbowBend,   2*elbowBend,   elbowBend+pi/2, pi/2];

% joystick object:
joy = vrjoystick(1);
trajGen = HebiTrajectoryGenerator(kin);
fkHome = kin.getForwardKinematics('endEffector',homePos);
endAngle = homePos(end) ;% angle of last joint with claw
xmax = [-.03; .25; .2; pi; pi; pi];
xmin = [-.85; -.25; -0.75; -pi; -pi; -pi];
dt = .001;

%% misc vars
gravity = [0; 0; -9.81];
axesThres  = .1;

EA = -SpinCalc('DCMtoEA123', fkHome(1:3,1:3), 1e-6, 0) * pi/180; % converts to RPY
EA(EA>pi) = EA(EA>pi) - 2*pi;
EA(EA<-pi) = EA(EA<-pi) + 2*pi;
x0 = [fkHome(1:3,4);  EA.'];


%% admittance gains

M0 = diag([1;1;1;   1;1;1]*.1);
B0 = diag([1;3;2;     1;1;1  ]*1  );
K0 = diag([.5;1;.5;     1;1;1  ]*10 );
M = M0; B = B0; K = K0;
FMult0 = .6*ones(6,1);
FMult0(3) = .4;
FMult = FMult0;

x = x0;
dx = zeros(6,1);

fbk = group.getNextFeedback();
disp(['Mean voltage: ' num2str(mean(fbk.voltage), '%2.1f'), ' V']);

positions = [fbk.position; ...
    homePos];
lastPosition = homePos;
trajectory = trajGen.newJointMove(positions);
trajGen.setSpeedFactor(1);
running = 1;
moveToInitialPos; % Moves the arm!

if plottingLog
    close all;
    figure;
    xPlot = plot(0,0,'r'); hold on;
    yPlot = plot(0,0,'g'); hold on;
    zPlot = plot(0,0,'b'); hold on;
    x0Plot = plot(0,0,':r'); hold on;
    y0Plot = plot(0,0,':g'); hold on;
    z0Plot = plot(0,0,':b'); hold on;
end
xLog = x;
x0Log = x0;
tLog = 0;
FLog = zeros(6,1);

if plottingArm
    figure;
    armPlot = plot3(0,0,0, 'k'); hold on;
    xScatter = scatter3(x(1), x(2), x(3), 'k', 'filled');
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end


%% pose filter on base
poseFilter = HebiPoseFilter();
poseFilter.setYaw(0); % (optional) set yaw to origin


% manually hold for a second
tic;
while toc<1
    group.set(cmd);
    pause(.001);
end

% set x based on initial pose?
% fbk = group.getNextFeedback();
% fk = kin.getForwardKinematics('Output',fbk.position);
% x(1:3) = fk(1:3,4);

%% main loop
tic;
running = 1;
while running
    t= toc;
    % get feedback
    fbk = group.getNextFeedback();
    
    % update pose filter
    accels = [fbk.accelX(1), fbk.accelY(1), fbk.accelZ(1)];
    gyros = [fbk.gyroX(1), fbk.gyroY(1), fbk.gyroZ(1)];
    poseFilter.update(accels, gyros, fbk.time);
    pose = poseFilter.getPose();
    
    % automatically determine direction of gravity
    %     gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
    gravityNow = pose(1:3,1:3)* -gravity;
    %     gravityNow =gravity;
    
    % gravity compensation efforts
    efforts = kin.getGravCompEfforts(fbk.position, gravityNow);
    
    
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
    
    % joystick commands: move gains
    if buttons(1)||buttons(2)
        K(1:7:end) = K(1:7:end) + (buttons(1) - buttons(2))*.1;
        disp(['K: ' ,num2str(K(1:7:end))])
    end
    if buttons(3)||buttons(4)
        B(1:7:end) = B(1:7:end) + (buttons(3) - buttons(4))*.1;
        disp(['B: ' ,num2str(B(1:7:end))])
    end
    
    % joystick commands: move x0
    x0(1) = x0(1) +  axes(2)*.02; % x
    x0(2) = x0(2) +  axes(1)*.005; % y
    x0(3) = x0(3) + -axes(4)*.02; % z
    %     x0(4) = x0(4) + (buttons(7)-buttons(8))*.01; % roll
    x0 = max(x0, xmin);
    x0 = min(x0, xmax);
    
    
    % get fk
    fk = kin.getForwardKinematics('Output',fbk.position);
    
    % get jacobians
    J = kin.getJacobian('endEffector',fbk.position);
    
    % Tau = J* .' * F
    F = zeros(6,1);
    F(1:3) = pinv(J(1:3,:).')* (fbk.effort-efforts).';
    
    %    	% here's where things get complicated, with directional compliance
    if directionalAdmittanceOn
        M=  M0;
        B = B0; 
        K = K0;
        FMult = FMult0;
        
%         % change damping
%         if dx(1)>0 % allow moving close to wall,
%             B(1,1) = 0.5*B0(1,1);
%         else
%             B(1,1) = B0(1,1);
%         end


        
%         if norm(dx(1:3))> 2
%              B(1,1) = B0(1,1)*20;
%               B(2,2) = B0(2,2)*20;
%                B(3,3) = B0(3,3)*20;
%         else
%              B(1,1) = B0(1,1);
%               B(2,2) = B0(2,2);
%                B(3,3) = B0(3,3);
%         end
        

        % change stiffness
        if x(1)<fkHome(1,4)-.1 % should be around -.45

            K(1,1) = 50*K0(1,1);% x dir
            FMult(1) = -FMult0(1)*.2; % should make it slightly anti-compliant

        else
            K(1,1) = K0(1,1);
            FMult(1) = FMult0(1);

        end
        
        if norm(F(1:3))>12
            FMult = 0.2*FMult;
            B = B0*5;
        end
        
    end
    
    
    % admittance time!
%     ddx = M\(-B*dx - K*(x-x0) + -F.*FMult0); % regular old linear spring
    %     ddx = M\(-B*dx - K*(x-x0) + F*0); % linear spring no forcing
    ddx = M\(-B*dx.*abs(dx) - K*(x-x0) - F.*FMult); % stiffening nonlinear spring
    dx = dx + ddx*dt;
    x = x + dx*dt;
    
    % put some caps on the positions?
    % max ... min ...
    %%% PROBLEM is this causes oscillations around these maxes??
    overx = x> xmax;
    underx = x<xmin;
    x(overx) = xmax(overx);
    x(underx) = xmin(underx);
    dx(overx) = 0;
    dx(underx) = 0;
    
    
    % convert to commands for arm
    tipAx =  fk(1:3,1:3,1)* [1;0;0];
    position = kin.getInverseKinematics('xyz',x(1:3),...
        'TipAxis', tipAx,...
        'InitialPositions',fbk.position,...
        'MaxIterations',1000);
    
    % safety limits
    position = min(position, thMax);
    position = max(position, thMin);
    

%     % low pass position
%     position = alpha*lastPosition + (1-alpha)*position;
%     lastPosition = position;
    if povs==90
endAngle = endAngle+.05;
elseif povs==270
endAngle = endAngle-.05;
end
position(end) = endAngle;


    cmd.position = position;
    cmd.velocity = nan(1,n);
%     cmd.effort = efforts;
    cmd.effort = nan(1,n); %% TODO: Why does this stop it from holding positions tightly?
    group.set(cmd);
    claw.set(cmdClaw);
    
    disp(['x0: ', num2str(round(x0(1:3).',2)), ...
        ' | x: ' ,num2str(round( x(1:3).',2)),...
        ' | F: ' ,num2str(round( F(1:3).',2)),...
        ])

    xLog = [xLog, x];
    x0Log = [x0Log, x0];
    tLog = [tLog; t];
    FLog = [FLog, F];
    
    if plottingLog||plottingArm
        if plottingLog
            set(xPlot, 'xdata', tLog, 'ydata', xLog(1,:));
            set(yPlot, 'xdata', tLog, 'ydata', xLog(2,:));
            set(zPlot, 'xdata', tLog, 'ydata', xLog(3,:));
            set(x0Plot, 'xdata', tLog, 'ydata', x0Log(1,:));
            set(y0Plot, 'xdata', tLog, 'ydata', x0Log(2,:));
            set(z0Plot, 'xdata', tLog, 'ydata', x0Log(3,:));
        end
        if plottingArm
            xyz = squeeze(fk(1:3,4,:));
            set(armPlot, 'xdata', xyz(1,:), 'ydata', xyz(2,:), 'zdata', xyz(3,:));
            axis equal;
            set(xScatter, 'xdata', x(1), 'ydata', x(2), 'zdata', x(3));
        end
        pause(0.0001);
    else
        pause(0.001);
    end
    
end

cmd.position = nan(1,n);
cmd.velocity = nan(1,n);
cmd.effort = nan(1,n);
group.set(cmd);

