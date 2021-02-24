% pushTorqueMode
% push in a direction specified with torque control

remainInMode = 1;
while remainInMode
    [axes,buttons,povs] = read(joy);
    
    if any(buttons(1:numGoals))
        newGoalNum = find(buttons(1:numGoals));
        goalNum = newGoalNum(1); % only take the lowest number pressed
        remainInMode = 0;
    end
    running = ~any(buttons(9:10));
    if ~running
        remainInMode = 0;
    end
    
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
    pos =  fbk.position;
     end
    
         J = kin.getJacobian('output', pos);
    
        
    J_part = J(1:3, :, end);
    F = directionToPush * .01;
    tauPush = J_part.' * F;

    
    if sendCommands
    
         cmd.position = NaN(1,n);
        cmd.velocity = NaN(1,n);
        cmd.effort = efforts + tauPush.' ;
        robot.set(cmd);
    end
    
    if animate
        plt.plot(pos)
    else
        pause(.01);
    end
    
    
end
lastPos = pos;