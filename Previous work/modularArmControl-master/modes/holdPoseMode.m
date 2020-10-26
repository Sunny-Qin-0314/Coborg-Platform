% holds arm in place


% tts('Hold pose','Microsoft Zira Desktop - English (United States)');
pos = lastPos;

disp(pos);

remainInMode = 1;
while remainInMode
    if sendCommands
        try
            fbk = robot.getNextFeedback();
        catch err
            disp(err.message)
        end
    end
    
    
    checkForNewMode;
    
        if sendCommands
        fkNow = kin.getForwardKinematics('Output',fbk.position);
    else
        fkNow = kin.getForwardKinematics('Output', pos); 
        end
    
    if sendCommands
        
        %% gravity comp torques
        % Compensate gravity at current position
%         fbk = robot.getNextFeedback();
        % automatically determine direction of gravity
        gravityVec = [fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
        % dealing with non identity base frames
        baseFrame = kin.getBaseFrame();
        gravityVec = baseFrame(1:3,1:3)' * gravityVec;
        efforts = kin.getGravCompEfforts(fbk.position, gravityVec);
        
        cmd.position = pos;
        cmd.velocity = NaN(1,n);
        cmd.effort =  efforts;
        try
            robot.set(cmd);
        catch err
            disp(err.message);
        end
    end
    
      if buttons(numGoals+1)&&(numGoals<nButtons-4)

            numGoals = numGoals+1;
            th = [th, lastPos.'];
            disp(['added goal: ',num2str(numGoals)]);
            goalNum = numGoals;
            lastGoalNum = goalNum;
            
      end
        
    
    if animate
        set(fkPlot, 'xdata', fkNow(1,4,:),'ydata', fkNow(2,4,:) ,'zdata', fkNow(3,4,:) );
    else
        pause(.01);
    end
end
lastPos = pos;
