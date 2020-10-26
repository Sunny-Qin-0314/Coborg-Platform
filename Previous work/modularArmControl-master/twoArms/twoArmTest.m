% tests with two arms

setupTwoArms;

while true
    
    if sendCommands
   fbkRight = robotRight.getNextFeedback();
   fbkLeft = robotLeft.getNextFeedback();
%    
%    % right master ,left slave
%    cmdLeft.position = -fbkRight.position;
%    rightGravCompEfforts = kinRight.getGravCompEfforts(fbkRight.position, [0 0 -1]);
%    leftGravCompEfforts = kinLeft.getGravCompEfforts(cmdLeft.position, [0 0 -1]);
   
      % leftmaster ,right  slave
%    cmdRight.position = -fbkLeft.position;
%    rightGravCompEfforts = kinRight.getGravCompEfforts(cmdRight.position, [0 0 -1]);
%    leftGravCompEfforts = kinLeft.getGravCompEfforts(fbkLeft.position, [0 0 -1]);
%    
   
   % hybrid master slave
   cmdRight.position = -fbkLeft.position;
   cmdLeft.position  = -fbkRight.position;
   rightGravCompEfforts = kinRight.getGravCompEfforts(cmdRight.position, [0 0 -1]);
   leftGravCompEfforts = kinLeft.getGravCompEfforts(cmdLeft.position, [0 0 -1]);
   
   
   cmdLeft.effort = leftGravCompEfforts;
   cmdRight.effort = rightGravCompEfforts;
   robotLeft.send(cmdLeft);
   robotRight.send(cmdRight);
    end
    
    pause(.01);
end


