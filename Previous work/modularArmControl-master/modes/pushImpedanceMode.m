% pushMode
% push in a linear direction with an impedance controller

% tts(['Pushing ' pushString],'Microsoft Zira Desktop - English (United States)');


if sendCommands
    fkNow = kin.getForwardKinematics('Output',fbk.position);
else
    fkNow = kin.getForwardKinematics('Output', pos);
    
end

% I set all these gains by hand unfortunately
x = fkNow(1:3, 4, end);
x0 = x + directionToPush*0.2;
xLast = x - directionToPush*0.2;
xdot = x-xLast;
M = .1*eye(3);
B = 3*eye(3);
K = 60*eye(3);
% dt = .01;
% fmult = abs(directionToPush)*.5;

remainInMode = 1;
firstTime = 1;
while remainInMode
   
    checkForNewMode;
    
    
    
    if sendCommands
        try
            fbk = robot.getNextFeedback();
        catch err
            disp(err.message)
        end
        %% gravity comp torques
        % Compensate gravity at current position
        % automatically determine direction of gravity
        gravityVec = [fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
        % dealing with non identity base frames
        baseFrame = kin.getBaseFrame();
        gravityVec = baseFrame(1:3,1:3)' * gravityVec;
        efforts = kin.getGravCompEfforts(fbk.position, gravityVec);
        
        fkNow = kin.getForwardKinematics('Output',fbk.position);
        
        J = kin.getJacobian('EndEffector', fbk.position);
        
    else
        
        J = kin.getJacobian('EndEffector', pos);
        
    end
    
    
    x = fkNow(1:3, 4, end);
    xdot = x- xLast;
    if firstTime % just ignore this hack to make it behave smoothly when starting up
        firstTime = 0;
        F3 = K*(x0 - x) - 5*B*xdot;
        
        %% TO DO: use baseFrame to change so that push direction is relative to gravity
            F6 = [F3; 0; 0; 0]; % wrench at end effector desired
    tau = J.' * F6*.1 + efforts.';
    else % the normal method: the force is PD related to the position of the end effector
        F3 = K*(x0 - x) - B*xdot;
            F6 = [F3; 0; 0; 0]; % wrench at end effector desired
    tau = J.' * F6 + efforts.'; % Then the torque needed to acheive that force is made here
    end % "efforts" is from getgravitycompensations

    
    
    if sendCommands
        
        cmd.position = NaN(1,n);
        cmd.velocity = NaN(1,n);
        cmd.effort = tau.';
        robot.set(cmd);
    end
    
    if animate
        set(fkPlot, 'xdata', fkNow(1,4,:),'ydata', fkNow(2,4,:) ,'zdata', fkNow(3,4,:) );
    else
        pause(.01);
    end
    xLast = x;
    
end
lastPos = pos;
