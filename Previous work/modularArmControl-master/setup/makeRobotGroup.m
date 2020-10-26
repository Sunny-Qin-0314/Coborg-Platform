% makeRobotGroup
% makes the hebi group and cmd structs

% robot = HebiLookup.newConnectedGroupFromName('*', 'base_temp');
robot = HebiLookup.newGroupFromNames('*', {'base_temp', 'shoulder_temp', 'elbow_temp'});
n = robot.getNumModules;

cmd = CommandStruct();
cmd.position = NaN(1,n);
cmd.velocity = NaN(1,n);
cmd.effort = NaN(1,n);
robot.set(cmd);