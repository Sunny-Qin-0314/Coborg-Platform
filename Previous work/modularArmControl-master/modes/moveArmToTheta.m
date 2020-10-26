% sends the robot from wherever it was last, to its new position
% select an end point to go to

% TO DO: scale trajectory duration by the distance travelled by the
% most-moved joint.

remainInMode = 1;
% tts(['Pose ' num2str(goalNum)],'Microsoft Zira Desktop - English (United States)');

positionEnd = max(positionEnd, thmin);
positionEnd = min(positionEnd, thmax);


positions = [lastPos; ...
    positionEnd];

trajectory = trajGen.newJointMove(positions);
%     trajectory = trajGen.newLinearMove(positions);

trajGen.setSpeedFactor(2);

% Manually execute position/velocity/torque trajectory
t0 = tic();
t = toc(t0);
while (t < trajectory.getDuration() && running && remainInMode)
  
    checkForNewMode;
    
    if sendCommands
        try
            fbk = robot.getNextFeedback();
        catch err
            disp(err.message)
        end
    end
    
    % get state at current time interval
    t = toc(t0);
    [pos, vel, accel] = trajectory.getState(t);
    
    if sendCommands
        fkNow = kin.getForwardKinematics('Output',fbk.position);
    else
        fkNow = kin.getForwardKinematics('Output', pos); 
    end
    
    if sendCommands
        
        % compensate for gravity
        gravityVec = [fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
        % dealing with non identity base frames
        baseFrame = kin.getBaseFrame();
        gravityVec = baseFrame(1:3,1:3)' * gravityVec;
        
        gravCompEffort = kin.getGravCompEfforts(fbk.position, gravityVec);
        
        % compensate for accelerations
        accelCompEffort = kin.getDynamicCompEfforts(...
            fbk.position, ... % used for jacobian
            pos, vel, accel);
        
        
        % send to hardware
        cmd.position = pos;
        %             cmd.velocity = vel; %THIS BREAKS MODULES, NOT SURE WHY
        cmd.effort = gravCompEffort + accelCompEffort;
        try
            robot.set(cmd);
        catch err
            disp(err.message);
        end
    end
    
    
    if animate
        set(fkPlot, 'xdata', fkNow(1,4,:),'ydata', fkNow(2,4,:) ,'zdata', fkNow(3,4,:) );
    else
        pause(.01);
    end
    
end

lastPos = pos;


