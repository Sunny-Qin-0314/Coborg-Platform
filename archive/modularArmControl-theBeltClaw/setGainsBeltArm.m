gains = GainStruct();

gains.controlStrategy= [3 3 3 3 3];
gains.mStopStrategy= [0 0 0 0 0];
% gains.positionKp= [2 3 1.5 1.5 1.5]; original defaults
gains.positionKp= [2 3 3 1.5 1.5];
% gains.positionKp= [2 8 12 1.5 1.5];
% gains.positionKi= [0 0 0 0 0]; % original
gains.positionKi= [1 1 1 1 1]*1000*.001; 

% gains.positionKd= [0 0 0 0 0]; % default
% gains.positionKd= [.001 .001 .001 .001 .001]*5;
gains.positionKd= [.001 .01 .005 .001 .001]*5;
gains.positionFF= [0 0 0 0 0];
gains.positionDeadZone= [0 0 0 0 0];
gains.positionIClamp= [0.25 0.25 0.25 0.25 0.25];
gains.positionPunch= [0 0 0 0 0];
gains.positionMinTarget= [-Inf -Inf -Inf -Inf -Inf];
gains.positionMaxTarget= [Inf Inf Inf Inf Inf];
gains.positionMinOutput= [-1 -1 -1 -1 -1];
gains.positionMaxOutput= [1 1 1 1 1];
gains.positionTargetLowpassGain= [1 1 1 1 1];
gains.positionOutputLowpassGain= [1 1 1 1 1];
gains.positionDOnError= [1 1 1 1 1];

gains.velocityKp= [0.03 0.03 0.05 0.05 0.05];
gains.velocityKi= [0 0 0 0 0];
gains.velocityKd= [0 0 0 0 0];
gains.velocityFF= [1 1 1 1 1];
gains.velocityDeadZone= [0 0 0 0 0];
gains.velocityIClamp= [0.25 0.25 0.25 0.25 0.25];
gains.velocityPunch= [0 0 0 0 0];
gains.velocityMinTarget= [-3.435 -3.435 -1.503 -1.503 -1.503];
gains.velocityMaxTarget= [3.435 3.435 1.503 1.503 1.503];
gains.velocityMinOutput= [-1 -1 -1 -1 -1];
gains.velocityMaxOutput= [1 1 1 1 1];
gains.velocityTargetLowpassGain= [1 1 1 1 1];
gains.velocityOutputLowpassGain= [0.75 0.75 0.75 0.75 0.75];
gains.velocityDOnError= [1 1 1 1 1];

gains.effortKp= [0.1 0.1 0.25 0.25 0.25];
gains.effortKi= [0 0 0 0 0];
gains.effortKd= [0.0001 0.0001 0.001 0.001 0.001];
gains.effortFF= [1 1 1 1 1];
gains.effortDeadZone= [0 0 0 0 0];
gains.effortIClamp= [0.25 0.25 0.25 0.25 0.25];
gains.effortPunch= [-0 -0 -0 -0 -0];
gains.effortMinTarget= [-25 -25 -20 -20 -20];
gains.effortMaxTarget= [25 25 20 20 20];
gains.effortMinOutput= [-1 -1 -1 -1 -1];
gains.effortMaxOutput= [1 1 1 1 1];
gains.effortTargetLowpassGain= [1 1 1 1 1];
gains.effortOutputLowpassGain= [0.9 0.9 0.9 0.9 0.9];
gains.effortDOnError= [0 0 0 0 0];

group.set('gains', gains);