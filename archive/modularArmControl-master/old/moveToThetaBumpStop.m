% sends the robot from wherever it was last, to its new position
% select an end point to go to


tts(['Pose ' num2str(goalNum)],'Microsoft Zira Desktop - English (United States)');

positions = [lastPos; ...
    positionEnd];

trajectory = trajGen.newJointMove(positions);
%     trajectory = trajGen.newLinearMove(positions);


% Manually execute position/velocity/torque trajectory
t0 = tic();
t = toc(t0);
while t < trajectory.getDuration()
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
        
        expectedEffort = cmd.effort;
        if any( abs(expectedEffort - fbk.effort )>3 )
            disp('too much torque:');
            disp(expectedEffort - fbk.effort);
            goalNum = 0;
            lastGoalNum = 0;
            break;
        end
        
        
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
        plt.plot(pos)
    else
        pause(.01);
    end
    
end

lastPos = pos;


