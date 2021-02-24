gains = GainStruct();
n= 1;
ones_n = ones(1,n);
gains.controlStrategy= ones_n*3;
gains.mStopStrategy= ones_n*0;
% gains.positionKp= [2 3 1.5 1.5 1.5]; original defaults
gains.positionKp= 3;
% gains.positionKp= [2 8 12 1.5 1.5];
% gains.positionKi=  ones_n*0; % original
gains.positionKi= ones_n*1000*.001; 

% gains.positionKd=  ones_n*0; % default
% gains.positionKd= [.001 .001 .001 .001 .001]*5;
gains.positionKd= .001;
gains.positionFF= ones_n*0;
gains.positionDeadZone=  ones_n*0;
gains.positionIClamp=  ones_n*.25;
gains.positionPunch=  ones_n*0;
gains.positionMinTarget=  ones_n*-Inf;
gains.positionMaxTarget=  ones_n*Inf;
gains.positionMinOutput=  ones_n*-1;
gains.positionMaxOutput=  ones_n*1;
gains.positionTargetLowpassGain=  ones_n*1;
gains.positionOutputLowpassGain=  ones_n*1;
gains.positionDOnError=  ones_n*1;

gains.velocityKp=  ones_n*0.03;
gains.velocityKi=  ones_n*0;
gains.velocityKd=  ones_n*0;
gains.velocityFF=  ones_n*1;
gains.velocityDeadZone=  ones_n*0;
gains.velocityIClamp=  ones_n*0.25;
gains.velocityPunch=  ones_n*0;
gains.velocityMinTarget=  ones_n*-3.435;
gains.velocityMaxTarget=  ones_n*3.435;
gains.velocityMinOutput=  ones_n*-1;
gains.velocityMaxOutput=  ones_n*1;
gains.velocityTargetLowpassGain=  ones_n*1;
gains.velocityOutputLowpassGain=  ones_n*0.75;
gains.velocityDOnError=  ones_n*1;

gains.effortKp=  ones_n*0.1;
gains.effortKi=  ones_n*0;
gains.effortKd=  ones_n*0.0001;
gains.effortFF=  ones_n*1;
gains.effortDeadZone=  ones_n*0;
gains.effortIClamp=  ones_n*0.25;
gains.effortPunch=  ones_n*0;
gains.effortMinTarget=  ones_n*-25;
gains.effortMaxTarget=  ones_n*25;
gains.effortMinOutput=  ones_n*-1;
gains.effortMaxOutput=  ones_n*1;
gains.effortTargetLowpassGain=  ones_n*1;
gains.effortOutputLowpassGain=  ones_n*0.9;
gains.effortDOnError=  ones_n*0;

group.set('gains', gains);