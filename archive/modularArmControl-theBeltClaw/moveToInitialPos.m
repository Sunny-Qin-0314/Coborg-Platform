% Manually execute position/velocity/torque trajectory

% automatically determine direction of gravity
fbk = group.getNextFeedback();
gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
% dealing with non identity base frames
baseFrame = kin.getBaseFrame();
gravity = baseFrame(1:3,1:3)' * gravityVec;

    
t0 = tic();
t = toc(t0);
while (t < trajectory.getDuration() && running)
    
    [axes,buttons,povs] = read(joy);
    running = ~any(buttons(9:12));
    
    
    fbk = group.getNextFeedback();
    
    
    % get state at current time interval
    t = toc(t0);
    [pos, vel, accel] = trajectory.getState(t);
    
    
    fkNow = kin.getForwardKinematics('Output',fbk.position);
    


    gravCompEffort = kin.getGravCompEfforts(fbk.position, gravity);
    
    % compensate for accelerations
    accelCompEffort = kin.getDynamicCompEfforts(...
        fbk.position, ... % used for jacobian
        pos, vel, accel);
    
    
    % send to hardware
    cmd.position = pos;
    %             cmd.velocity = vel; %THIS BREAKS MODULES, NOT SURE WHY
%     cmd.effort = gravCompEffort + accelCompEffort;
    cmd.effort = nan(1,n); 
    group.set(cmd);
    
    
    
    
    
    pause(.005);
    
    
end