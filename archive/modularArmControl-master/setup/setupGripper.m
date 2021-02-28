% setupGripper

gripper = HebiLookup.newConnectedGroupFromName('*', 'SA016');
cmdGripper = CommandStruct();
cmdGripper.position = NaN;
cmdGripper.velocity = NaN;
cmdGripper.effort = NaN;

fbkGripper = gripper.getNextFeedback();
cmdGripper.position = fbkGripper.position;
gripper.set(cmdGripper);

gripperMin = -.7;
gripperMax = .9;

%% TO DO:
% set gains
% add weight of gripper to model
gripperMass = .450; % kg

mass = 1; % [kg]
com = [1 0 0]; % 1 [m] in x
kin.setPayload(mass, 'CoM', com);

