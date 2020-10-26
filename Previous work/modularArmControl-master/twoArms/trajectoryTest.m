
setupTwoArms;
cmdRight.position = zeros(1,n);
robotRight.set(cmdRight);
pause(.5)
    fbk = robotRight.getNextFeedback();


% Create trajectory through multiple waypoints
nWaypoints = 10;
nJoints = kinRight.getNumDoF();
positions = [fbk.position; rand(nWaypoints-1, nJoints)*pi/2 ];
defaultVelocities = [ zeros(1, nJoints); nan(nWaypoints-2, nJoints);  zeros(1, nJoints)];
defaultAccels= [ zeros(1, nJoints); nan(nWaypoints-2, nJoints);  zeros(1, nJoints)];

traj = trajGenRight.newJointMove(positions,...
    'Velocities', defaultVelocities,...
    'Accelerations', defaultAccels);

% Execute trajectory in position and velocity
cmd = CommandStruct();
t0 = tic();
t = toc(t0);
while t < traj.getDuration()
    t = toc(t0);
    
    % React to something (e.g. position error or effort threshold)
    fbk = robotRight.getNextFeedback();
    
    % Get target state at current point in time
    [pos, vel, accel] = traj.getState(t);
    
    rightGravCompEfforts = kinRight.getGravCompEfforts(cmdRight.position, [0 0 -1]);
    rightDynamicCompEfforts = kinRight.getDynamicCompEfforts(...
        fbk.position, ...
        pos, ...
        vel, ...
        accel);
    
    
    if  norm(fbk.effort - (rightGravCompEfforts + rightDynamicCompEfforts))  > 0.5
        disp('Reacting to something...');
        pause;
    end
    
    % Command position/velocity
    cmdRight.position = pos;
    %     cmdRight.velocity = vel;
    cmdRight.effort = rightGravCompEfforts + rightDynamicCompEfforts;
    robotRight.send(cmdRight);
    
end

% Create trajectory w/ automatically determined time
        positions = [0 0; .75 .75; 1 1; ]*pi/2;
        trajectory = trajGenRight.newJointMove(positions);

        % Simple execution
        trajGenRight.executeTrajectory(robotRight, trajectory);
        fbkRight = robotRight.getNextFeedback();
%
%
%         % Execution with full options
%         gravityVec = -[fbkRight.accelX(1) fbkRight.accelY(1) fbkRight.accelZ(1)];
%         trajGenRight.executeTrajectory(robotRight, trajectory, ...
%             'GravityVec', gravityVec, ...
%             'DynamicsComp', true, ...
%             'EffortOffset', zeros(1, kin.getNumDoF));%, ...
% %             'Callback', @(time, fbk, cmd) cmd);
% % this might fix but does not right now:
% trajGenRight.executeTrajectory(robotRight, trajectory, ...
%             'GravityVec', gravityVec, ...
%             'DynamicsComp', true, ...
%             'EffortOffset', zeros(1, kin.getNumDoF), ...
%             'Callback', @(time, fbk, cmd)callbackNoVelocity);




% Visualize trajectory
figure(); hold on; grid on;
% Position / Velocity / Acceleration profile
tplot = 0:0.01:traj.getDuration();
[pos, vel, accel] = traj.getState(tplot);
plot(tplot, pos);
plot(tplot, vel);
% plot(t, accel);

% Superimpose target position waypoints
% tWaypoint = traj.getWaypointTime();
% states= traj.getState(0:.01:tWaypoint(end));
% plot(repmat((0:.01:tWaypoint(end)).', [1,nJoints]), states, 'b-');
% plot(repmat(tWaypoint, [1,nJoints]), traj.getState(tWaypoint), 'bo');

% Superimpose target position waypoints
tWaypoint = traj.getWaypointTime();
plot(tWaypoint, traj.getState(tWaypoint), 'bo');
% Axes labels
title('Joint Trajectory');
ylabel('Value [rad, rad/s, rad/s^2]');
xlabel('Time [s]');
% legend Position Velocity Acceleration