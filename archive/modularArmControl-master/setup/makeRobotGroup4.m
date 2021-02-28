% makeRobotGroup
% makes the hebi group and cmd structs

names = {'X-00088', 'X-00093', 'X-00003', 'X-00043'};
robot = HebiLookup.newGroupFromNames('*', names);
n = length(names);
cmd = CommandStruct();
cmd.position = NaN(1,n);
cmd.velocity = NaN(1,n);
cmd.effort = NaN(1,n);
robot.set(cmd);