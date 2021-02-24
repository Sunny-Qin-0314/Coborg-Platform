
gains = GainStruct();
ones_n = ones(1, n);

gains.controlStrategy= ones_n*3;
%                 mStopStrategy= [NaN NaN 0 0 0];

gains.positionKp= ones_n*1;
gains.positionKi= ones_n*1000*.25;
gains.positionKd= ones_n*.001*3;
gains.positionFF= ones_n*0;
gains.positionDeadZone= ones_n*0;
gains.positionIClamp= ones_n*0;
gains.positionPunch= ones_n*0;
gains.positionMinTarget= ones_n*-inf;
gains.positionMaxTarget= ones_n*inf;
gains.positionMinOutput= ones_n*-pi;
gains.positionMaxOutput= ones_n*pi;
gains.positionTargetLowpassGain= ones_n;
gains.positionOutputLowpassGain= ones_n;
gains.positionDOnError= ones_n;

gains.velocityKp= ones_n*.1;
gains.velocityKi= ones_n*0;
gains.velocityKd= ones_n*0;
gains.velocityFF= ones_n;
gains.velocityDeadZone= ones_n*0;
gains.velocityIClamp= ones_n*.25;
gains.velocityPunch= ones_n*0;
gains.velocityMinTarget= ones_n*-3;
gains.velocityMaxTarget= ones_n*3;
gains.velocityMinOutput= ones_n*-1;
gains.velocityMaxOutput= ones_n*1;
gains.velocityTargetLowpassGain= ones_n;
gains.velocityOutputLowpassGain= ones_n;
gains.velocityDOnError= ones_n*0;

gains.effortKp= ones_n*0;
gains.effortKi= ones_n*0;
gains.effortKd= ones_n*0;
gains.effortFF= ones_n;
gains.effortDeadZone= ones_n*0;
gains.effortIClamp= ones_n*0;
gains.effortPunch= ones_n*0;
gains.effortMinTarget= ones_n*-5;
gains.effortMaxTarget= ones_n*5;
gains.effortMinOutput= ones_n*-5;
gains.effortMaxOutput= ones_n*5;
gains.effortTargetLowpassGain= ones_n;
gains.effortOutputLowpassGain= ones_n;
gains.effortDOnError= ones_n*0;


robot.set('gains', gains);

               robot.setCommandLifetime(0); % so the commands don't expire
%                robot.setCommandLifetime(0.1); % so the commands expire

               % change fb frequency to reduce wireless traffic
                robot.setFeedbackFrequency(100);
               