% read in a csv file and look for new commands and their timestamps.
% use this information to change the "mode" variable if there's a match.

% tic
% read in data file
% data = csvread('exampleCSV.csv'); too slow
% data = textscan('exampleCSV.csv'); % too hard to get the output flags right
data = delimread('exampleCSV.csv', ',', 'text'); % seems pretty fast, and easy to use. 
% toc
% this might be slow if the file gets too long, though, so might have to
% reinvestigate later, since this needs to run very frequently
dataText = data.text;


% convert the most recent time stamp
timeStamp = dataText{end,1}; % exmaple: Tue Feb 17 10:00:18 2009
% see https://www.mathworks.com/help/matlab/ref/datenum.html for date
% conversion info
timeNum = datenum(timeStamp, 'ddd mmm dd HH:MM:SS yyyy');
word = dataText{end, 2};


% these keywords will be made at startup
keywords = {'hold', 'free', 'pose', 'push','new', 'follow', 'fold', ... % modes
    'up', 'down', 'left', 'right', 'forward', 'back',... % directions for push
    'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight',... % numbers for pose
    'teach', 'repeat', 'quit'}.'; % other kewords
% can add keywords during runtime with: keywords = [keywords; 'new'];

% hold: hold in place stiff
% free: move freely with grav comp
% pose <num>: move to pose number <num>
% push <dir>: push from current pose in the <dir> direction
% new pose: assign current pose as pose <max num>+1
% follow: first move arm smoothly to location of right hand, the copy with
%   IK floating ~3 inches above the hand.
% fold: move to the pose that is labelled as the fold pose
% new fold: assign current position (hold or pose) as the fold pose
% teach: free, but recording the motion
% repeat: play back the motion from most recent "teach"
% quit: exit program, kill arm.


% some other logic will be needed to match sets of words
% i.e. 'push' will only apply if paired with a direction,
% 'pose' will only apply if paired with a number, etc.



    
    


