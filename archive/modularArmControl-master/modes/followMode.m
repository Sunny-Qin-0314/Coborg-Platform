% follow with IK


% tts('Follow Mode','Microsoft Zira Desktop - English (United States)');
pos = lastPos;
fk = kin.getFK('EndEffector', pos);
xyz = fk(1:3,4);
thLast = pos;
% xyzMax = sum(lengths) + rb;
% xyzMin = rb - sum(lengths);
xyzMax = 1.5 + [dvars(1); 0; 0];
xyzMin = [dvars(1); 0; 0] - 1.5;

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
    
    axes(abs(axes)<.1) = 0;
    xyz = xyz + [-axes(2); axes(1); -axes(4)] * .01/2;
    xyz = max(xyz, xyzMin);
    xyz = min(xyz, xyzMax);
    
    
    pos = kin.getInverseKinematics('Xyz', xyz, 'InitialPositions', fbk.position);
    thLast = pos;
        
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
        
        
        pos = max(pos, thmin);
        pos = min(pos, thmax);
        
        cmd.position = pos;
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
    lastPos = pos;

end
lastPos = pos;
