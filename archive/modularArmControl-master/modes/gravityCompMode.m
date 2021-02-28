% gravityCompMode
% just gives the arm gravity compensation torques

disp(['Gravcomp: press button ', num2str(nButtons-1),  ' for Hold pose mode']);

% tts('Gravity compensation','Microsoft Zira Desktop - English (United States)');

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
        fbk = robot.getNextFeedback();
        % automatically determine direction of gravity
        gravityVec = [fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
        % dealing with non identity base frames
        baseFrame = kin.getBaseFrame();
        gravityVec = baseFrame(1:3,1:3).' * gravityVec;
        efforts = kin.getGravCompEfforts(fbk.position, gravityVec);
        
        
       
        pos = fbk.position;
        cmd.position = NaN(1,n);
        cmd.velocity = NaN(1,n);
        cmd.effort = efforts;
        
               
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
