%% Julian Whitman
% 1-29-2019

%% setup
% add paths
addpath(genpath(pwd));
addpath('C:\Users\medgroup01\Documents\Julian\hebi API\hebi1-2-2019');
close all;

%% flags
directionalAdmittanceOn = 1;
plottingLog = 1;


%% create  object
names = {'X-00468' };
group = HebiLookup.newGroupFromNames('*', names);
cmd = CommandStruct();
n = 1;
setGainsTest;

kin = HebiKinematics();
kin.addBody('X5-9');
kin.addBody('X5Link', 'ext', 0.2, 'twist', 0); % 500mm links -> .525

% joystick object:
joy = vrjoystick(1);
dt = .01/5;

%% misc vars
gravity = [0; 0; -9.81];
axesThres  = .1;


%% admittance gains

M0 = .001;
B0 = .01  ;
% K0 = 20 ;
K0 = 0 ;
M = M0; B = B0; K = K0;
FMult0 = 1;
FMult = FMult0;

x0 = 0;
x = x0;
dx = 0;

fbk = group.getNextFeedback();
disp(['Mean voltage: ' num2str(mean(fbk.voltage), '%2.1f'), ' V']);


running = 1;

if plottingLog
    close all;
    figure;
    xPlot = plot(0,0,'r'); hold on;
    x0Plot = plot(0,0,':r'); hold on;
 end
xLog = x;
x0Log = x0;
tLog = 0;
FLog = 0;

xmax = pi/2;
xmin = -pi/2;

%% pose filter on base
poseFilter = HebiPoseFilter();
poseFilter.setYaw(0); % (optional) set yaw to origin


% main loop
tic;
running = 1;
while running
    t= toc;
    % get feedback
    fbk = group.getNextFeedback();
    
    % update pose filter
    accels = [fbk.accelX(1), fbk.accelY(1), fbk.accelZ(1)];
    gyros = [fbk.gyroX(1), fbk.gyroY(1), fbk.gyroZ(1)];
    poseFilter.update(accels, gyros, fbk.time);
    pose = poseFilter.getPose();
    
    % automatically determine direction of gravity
    %     gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)].';
    gravityNow = pose(1:3,1:3)* -gravity;
    %     gravityNow =gravity;
    
    % gravity compensation efforts
    efforts = kin.getGravCompEfforts(fbk.position, gravityNow);
    F = fbk.effort(1);
    
    % get joystick buttons and map joystick to coordinates
    [axes,buttons,povs] = read(joy);
    
    axes(abs(axes)<axesThres) = 0;
    running = ~any(buttons(9:12));
 
    if directionalAdmittanceOn
       
%         % change damping
%         if dx(1)>0 % allow moving close to wall,
%             B(1,1) = 0.5*B0(1,1);
%         else
%             B(1,1) = B0(1,1);
%         end
        
          if abs(dx(1))>5
              B = 10;
          else
              B = B0;
          end

%         % change stiffness
%         if x(1)<0 % should be around -.4
% 
%             K = 50*K0;% x dir
%             B = B0;
%             FMult(1) = -FMult0(1)*.2; % should make it slightly anti-compliant
%         else
%             K(1,1) = K0(1,1);
%             B(1,1) = B0(1,1);
%             FMult(1) = FMult0(1);
%         end 
    end
    
    
    % admittance time!
    ddx = M\(-B*dx - K*(x-x0) + -F.*FMult0); % regular old linear spring
    %     ddx = M\(-B*dx - K*(x-x0) + F*0); % linear spring no forcing
%     ddx = M\(-B*dx.*abs(dx) - K*(x-x0) - F.*FMult); % stiffening nonlinear spring
    dx = dx + ddx*dt;
    x = x + dx*dt;
    
    % put some caps on the positions?
    % max ... min ...
    %%% PROBLEM is this causes oscillations around these maxes??
    overx = x> xmax;
    underx = x<xmin;
    x(overx) = xmax(overx);
    x(underx) = xmin(underx);
    dx(overx) = 0;
    dx(underx) = 0;
   

    cmd.position = x(1);
    cmd.velocity = nan(1,n);
    cmd.effort = efforts;
%     cmd.effort = nan(1,n); %% TODO: Why does this stop it from holding positions tightly?
    group.set(cmd);
    
    disp(['x0: ', num2str(round(x0(1).',2)), ...
        ' | x: ' ,num2str(round( x(1).',2)),...
        ' | dx: ' ,num2str(round( dx(1).',2)),...
        ' | F: ' ,num2str(round( F(1).',2)),...
        ])

    xLog = [xLog, x];
    x0Log = [x0Log, x0];
    tLog = [tLog; t];
    FLog = [FLog, F];
    
    if plottingLog
        if plottingLog
            set(xPlot, 'xdata', tLog, 'ydata', xLog(1,:));
          
            set(x0Plot, 'xdata', tLog, 'ydata', x0Log(1,:));
          
        end
      
        pause(0.0001);
    else
        pause(.001);
    end
    
end

cmd.position = nan(1,n);
cmd.velocity = nan(1,n);
cmd.effort = nan(1,n);
group.set(cmd);

