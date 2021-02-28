%sine wave test


tic;t=0;
while t<30
    t = toc;
  %% gravity comp torques
        % Compensate gravity at current position
        fbk = robot.getNextFeedback();
        % automatically determine direction of gravity
        gravityVec = [fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
        % dealing with non identity base frames
        baseFrame = kin.getBaseFrame();
        gravityVec = baseFrame(1:3,1:3)' * gravityVec;
        efforts = kin.getGravCompEfforts(fbk.position, gravityVec);
        
        pos = fbk.position;
        cmd.position = ones(1,n)*sin(t)*.5;
        cmd.velocity = NaN(1,n);
%         cmd.effort = efforts;    
        cmd.effort =  NaN(1,n);    

robot.set(cmd)

end
setToNaN