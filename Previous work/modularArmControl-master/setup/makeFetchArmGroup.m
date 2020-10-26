% makeRobotGroup
% makes the hebi group and cmd structs

robot = HebiLookup.newGroupFromNames('*', {'X-00674', 'X-00690','X-00599', 'X-00642', 'X-00654'});
n = robot.getNumModules;

cmd = CommandStruct();
cmd.position = NaN(1,n);
cmd.velocity = NaN(1,n);
cmd.effort = NaN(1,n);
robot.set(cmd);