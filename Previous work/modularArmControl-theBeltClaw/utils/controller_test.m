% controller_test

joy = vrjoystick(1);

tic;
while toc<30
% while toc>0
[axes, buttons, povs] = read( joy );
disp(num2str([axes, buttons, povs]));   


pause(0.02);

%================AXES================%
% axes = [leftX leftY rightX rightY]
% right and down are positive for both

%==============BUTTONS===============%
% buttons = [X A B Y LB RB LT RT Back Start LeftStick RightStick]
%           [1 2 3 4 5  6  7  8  9    10    11        12        ]

%================POVS================%
% povs = [Dpad]
% -1 means nothing is pressed
% 0 < pov < 360, depending on where you press it 
% 0 = up, 90 = right, 180 = down, 270 = left
% also reads diagonals as 45 degree angles

end