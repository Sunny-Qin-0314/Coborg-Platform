%% setup
% add paths
addpath(genpath(pwd));
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi1-2-2019');

%% create arm object
setupBeltArm;
elbowBend = pi/2 - pi/6;
homePos = [0, elbowBend,   2*elbowBend,   elbowBend+pi/2, pi/2];

% joystick object:
joy = vrjoystick(1);
trajGen = HebiTrajectoryGenerator(kin);


%% admittance gains
M0 = diag([1;1;0.1;   1;1;0.1]*.01);
B0 = diag([1;1;0;     1;1;0  ]*1  );
K0 = diag([1;1;0;     1;1;0  ]*10 );
M = M0; B = B0; K = K0;
x0 = [.5; 0; 0; 0; 0; 0];
xmax = [1; 1; 1; pi; pi; pi];
xmin = -xmax;
dt = .01;

%% misc vars
gravity = [0; 0; -9.81];
axesThres  = .1;
% get feedback
fbk = group.getNextFeedback();
% get fk
fk = kin.getForwardKinematics('endEffector',fbk.position);
EA = -SpinCalc('DCMtoEA123', fk(1:3,1:3), 1e-6, 0) * pi/180; % converts to RPY
EA(EA>pi) = EA(EA>pi) - 2*pi;
EA(EA<-pi) = EA(EA<-pi) + 2*pi;
x = [fk(1:3,4);  EA];
dx = zeros(6,1);

%% make sure that the arm moves to an elbow-down config first.
% convert to commands for arm
R = (R_z(x(6))*R_y(x(5))*R_x(x(4)));
position = kin.getInverseKinematics('xyz',x(1:3),'SO3',...
        R, 'InitialPositions', [0;0; -1; 0; 0],...
        'MaxIterations',1000);
positions = [fbk.position; ...
        position];

trajectory = trajGen.newJointMove(positions);
trajGen.setSpeedFactor(2);
moveToInitialPos; % Moves the arm!

%% main loop
running = 1;
while running
    % get feedback
    fbk = group.getNextFeedback();
    
    % get joystick buttons and map joystick to coordinates
    [axes,buttons,povs] = read(joy);
    
    axes(abs(axes)<axesThres) = 0;
    running = ~any(buttons(9:12));
    
    % use joystick commands: claw
    % open/close claw
    if buttons(5) % close
        cmdClaw.torque = 5; % change to good torque close
        cmdClaw.position = nan;
    elseif buttons(6) % open
        cmdClaw.torque = nan;
        cmdClaw.position = 0; % change to good position for open
    end

    % joystick commands: move x0
    x0(1) = x0(1) + axes(1)*.01; % x
    x0(2) = x0(2) + axes(2)*.01; % y
    x0(3) = x0(3) + axes(3)*.01; % z
    x0(4) = x0(4) + (buttons(7)-buttons(8))*.01; % roll   
    x0 = max(x0, xmin);
    x0 = min(x0, xmax);
    
    
    % get fk 
    fk = kin.getForwardKinematics('endEffector',fbk.position);
    
    % get jacobians
    J = kin.getJacobian('endEffector',fbk.position);
    
    % F = J* Tau.'
    F = J*cmd.torque;
   
   	% here's where things get complicated, with directional compliance
	if 0
	
	% change damping
	if dx(1)>0 % allow moving close to wall,
		B(1,1) = 0;
	else
		B(1,1) = B0(1,1);
	end
    if dx(5)>0 % allow tilting to wall,
		B(5,5) = 0;
	else
		B(5,5) = B0(5,5);
	end
	
	% change stiffness
	if x(1)>.5
		K(1,1) = 5*K0(1,1);
	else
		K(1,1) = K0(1,1)
	end
	
	end
    % admittance time!
   % ddx = M\(-B*dx - K*(x-x0) + F); % regular old linear spring	
    ddx = M\(-B*dx.*abs(dx) - K*(x-x0) + F); % stiffening nonlinear spring
    dx = dx + ddx*dt;
    x = x + dx*dt;

    % put some caps on the positions?
    % max ... min ...
    % check arm physical parameters, keep from going elbow up
    overx = x> xmax;
    underx = x<xmin;
    x0(overx) = xmax(overx);
    x0(underx) = xmin(underx);
    dx(overx) = 0;
    dx(underx) = 0;
    
    % convert to commands for arm
    R = (R_z(x(6))*R_y(x(5))*R_x(x(4)));
    position = kin.getInverseKinematics('xyz',x(1:3),'SO3',...
            R, 'InitialPositions',fbk.position,...
            'MaxIterations',1000);
    
    cmd.position = posiiton;
    cmd.velocity = nan(n,1);
    cmd.torque = nan(n,1);
    group.set(cmd);
    claw.set(cmdClaw);
    
    pause(dt);
    
end

cmd.position = nan(1,n);
cmd.velocity = nan(1,n);
cmd.effort = nan(1,n);
group.set(cmd);

