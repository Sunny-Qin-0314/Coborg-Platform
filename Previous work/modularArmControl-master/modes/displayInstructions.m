% displays the instructions

disp('Instructions.');
disp(['Buttons 1 - ', num2str(nButtons-4), ': Moves arm to pose number pressed.']);
disp('  - only buttons with poses assigned will work');
disp('  - press the next highest button unassigned to assign the current arm pose as that button.');
disp(['Button ', num2str(nButtons-1), ' (left joystick press in): Hold current pose'])
disp(['Button ', num2str(nButtons), ' (right joystick press in): Gravity compensation for teaching new poses'])
disp(['Button ', num2str(nButtons-3), ' or ', num2str(nButtons-2), ': Quit/exit program.'])
disp('D-pad: push up/down/forward');
disp(['Buttons ', num2str(nButtons-5), 'and', num2str(nButtons-4), ' at the same time enters IK control.']);
disp('  - use joysticks to direct end effector');
disp('  - Be careful-- no safety checks, not stable');