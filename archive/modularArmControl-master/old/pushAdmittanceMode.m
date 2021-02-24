% pushMode
% push in a linear direction with an admittance controller

fk = kin.getForwardKinematics('Output', pos);
x0 = fk(1:3, 4, end) + directionToPush*0.05;
if sendCommands
    
    fkNow = kin.getForwardKinematics('Output',fbk.position);
else
    fkNow = fk;
end
x = fkNow(1:3, 4, end);
xdot = [0;0;0];
M = .1*eye(3);
B = 20*eye(3);
K = 30*eye(3);
dt = .01;
fmult = abs(directionToPush)*.5;

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
        
        
        % tau = J^T * F, xdot= J * thdot
        % so F = (J^T)^+ * tau
        tau = fbk.effort.' - efforts.';
        J = kin.getJacobian('output', fbk.position);
        
    else
        tau = zeros(n,1);
        J = kin.getJacobian('output', pos);
    end
    
    J_part = J(1:3, :, end); % tau = J.' * F
    F = pinv(J_part.') * tau;
    xddot=  M\( -B*xdot - K*(x-x0) - F.*fmult);
    xdot = xddot*dt + xdot;
    x = x + xdot*dt;
    
    % cap params
    for coord = 1:3
        if (x(coord) - x0(coord))>.5
            x(coord) = x0(coord) + .5;
            xdot(coord) = 0;
        end
        if (x(coord) - x0(coord))<-.5
            x(coord) = x0(coord) - .5;
            xdot(coord) = 0;
        end
    end
    

%         % alternate method: jacobian transpose IK
%     if sendCommands
%         fk_posLast = kin.getFK('EndEffector', fbk.position );
%     else
%         fk_posLast = kin.getFK('EndEffector', pos );
%     end
%     dth =  J_part.' * (x - fk_posLast(1:3,4));
%     newPos = pos + dth.';
%     pos = newPos;
    


         if sendCommands
            newPos = kin.getInverseKinematics('xyz', x, 'InitialPositions', fbk.position);
        else
            newPos = kin.getInverseKinematics('xyz', x, 'InitialPositions', pos);
        end
            fk_pos = kin.getFK('EndEffector', newPos );
                dx = x -  fk_pos(1:3,4);
        if norm(dx) < .2
            pos = newPos;
        else
            disp(['out of range: fk= ' num2str(fk_pos(1:3,4).'), ' x_d= ', num2str(x.')]);
        end
    
    
    
    
    
    if sendCommands
        
        cmd.position = pos;
        cmd.velocity = NaN(1,n);
        cmd.effort = efforts;
        robot.set(cmd);
    end
    
    if animate
        plt.plot(pos)
    else
        pause(.01);
    end
    
    
end
lastPos = pos;
