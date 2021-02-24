# Creates kinematic object and has some utilities to use with it.
# For instance, gravity compensation torques.
# uses Hebi Python API,  http://docs.hebi.us/docs/python/1.0.1/hebi/py_api.html

import hebi
import numpy as np
from numpy import pi, cos, sin

#Creates 4x4 transformation matrix from rotation and position
def TRp(R, p):
  if p.shape[0]==1:
      p = p.T
  mid = np.hstack((R,p))
  out = np.vstack((mid,np.array([0.0,0.0,0.0,1.0])))
  return np.matrix(out)
  
#3x3 Rotation Matrices
def R_x(angle):
        DCM = np.matrix([[1,       0,            0     ],
                         [0,   cos(angle),  -sin(angle)],
                         [0,   sin(angle),   cos(angle)]])
        return DCM
        
def R_y(angle):
        DCM = np.matrix([[cos(angle),  0,  sin(angle)],
                        [     0,       1,       0    ],
                        [-sin(angle),  0,  cos(angle)]])
        return DCM

def R_z(angle):
        DCM = np.matrix([[cos(angle), -sin(angle),  0],
                         [sin(angle),  cos(angle),  0],
                         [    0,           0,       1]])
        return DCM

#Main function, sets up variables and creates kinematics object and joint_mask
def make_backpack_kin():
#    
#    #Change this list for application of diffferent configurations
#    module_types = ['BaseModuleZRY', 'LinkModule', \
#        'RotJointModule',  'LinkModule',\
#        'RotJointModule',  'LinkModule',\
#        'RotJointModule', 'EndEffectorVarLXModule']
#    
#        
#    #Values obtained from rpevious versions of the code
#    #(I believe these should be the same length as nModules)
#    dvars = [ [0.0065], [0.4000, 1.3343 - pi/2], [], [0.2317,2.1630], 
#             [], [0.2048, -3.7736], [], [0.2559] ] # empty lists if there is not a number associated with that dvar
#    kin, joint_mask = make_kin(module_types, dvars)
    
    
    kin = hebi.robot_model.RobotModel()
    #    module_types = ['BaseModuleZRY', 'LinkModule', \
    #        'RotJointModule',  'LinkModule',\
    #        'RotJointModule',  'LinkModule',\
    #        'RotJointModule', 'EndEffectorVarLXModule']
    #dvars = [ [0.0065], [0.4000, 1.3343 - pi/2], [], [0.2317,2.1630], 
    #             [], [0.2048, -3.7736], [], [0.2559] ] # empty lists if there is not a number associated with that dvar
    outputTransform = TRp(R_x(pi/2), np.matrix([0.,0.,0.0065])) 
    # can add other base configs later
    kin.base_frame = outputTransform
    kin.add_link('X5', 0.4000, (1.3343 - pi/2)%(2*pi))
    kin.add_actuator('X5-9')
    kin.add_link('X5', 0.2317, 2.1630)
    kin.add_actuator('X5-9')
    kin.add_link('X5', 0.2317, 2.1630)
    kin.add_actuator('X5-9')
    outputTransform = TRp(np.eye(3), np.matrix([0.2559, 0., 0.]))
    mass = .3
    com = TRp(np.eye(3),  np.matrix([0.2559, 0., 0.])/2)
    kin.add_rigid_body(com, [0.,0.,0.,0.,0.,.0], mass, outputTransform, False)
    
    joint_mask = [0, 1, 0, 1, 0, 1, 0]
    
    return kin, joint_mask
    
# repackaged this function for use with other types of robots in the future.
# Note: I commented out the module types that are not used right now since I did not test/debug them yet
def make_kin(module_types, dvars):
    
    joint_mask = []
    
#    nModules = len(module_types)
#    DVars = np.array([[1], [2], [0], [2], [0], [2], [0], [1]])
    #    currentDVar = 0

    kin = hebi.robot_model.RobotModel()


    for i in range (len(module_types)):
#        dvarsNow = dvars[currentDVar + np.arange(0, DVars[i])]
        dvarsNow = dvars[i]
        
        #Base Modules
        if module_types[i] == 'BaseModule':
            outputTransform = np.eye(4)
            kin.base_frame = outputTransform
            
#        elif module_types[i] == 'BaseModuleX':
#            outputTransform = TRp(np.eye(3), np.matrix([dvarsNow[0],0,0])) 
#            # can add other base configs later
#            kin.base_frame = outputTransform
#            
#        elif module_types[i] == 'BaseModuleZ':
#            outputTransform = TRp(np.eye(3), np.matrix([0,0,dvarsNow[0]])) 
#            # can add other base configs later
#            kin.base_frame = outputTransform
#
#        elif module_types[i] == 'BaseModuleXRX': 
#        # varies in X direction, joint rotation axis in the x direction
#            outputTransform = TRp(R_y(pi/2), np.matrix([dvarsNow[0],0,0])) 
#            # can add other base configs later
#            kin.base_frame = outputTransform
#
#        elif module_types[i] == 'BaseModuleXRY': 
#        # varies in X direction, joint rotation axis in the Y direction
#            outputTransform = TRp(R_x(-pi/2), np.matrix([dvarsNow[0],0,0])) 
#            # can add other base configs later
#            kin.base_frame = outputTransform

        elif module_types[i] == 'BaseModuleZRY':
            outputTransform = TRp(R_x(pi/2), np.matrix([0.,0.,dvarsNow[0]])) 
            # can add other base configs later
            kin.base_frame = outputTransform

#        elif module_types[i] == 'BaseModuleZRX':
#            outputTransform = TRp(R_y(pi/2), np.matrix([0,0,dvarsNow[0]])) 
#            # can add other base configs later
#            kin.base_frame = outputTransform
#                                      
#        elif module_types[i] == 'BaseModuleXZ':
#            outputTransform = TRp(R_x(pi), np.matrix([dvarsNow[0], 0,\
#            dvarsNow[1]])) # can add other base configs later
#            kin.base_frame = outputTransform
#                                      
#        elif module_types[i] == 'BaseModuleXZRX': 
#        # varies in X Z direction, joint rotation axis in the x direction
#            outputTransform = TRp(R_y(pi/2), np.matrix([dvarsNow[0],0,\
#            dvarsNow[1]])) # can add other base configs later
#            kin.base_frame = outputTransform
#
#        elif module_types[i] == 'BaseModuleXYZ':
#            outputTransform = TRp(np.eye(3),\
#            np.array(np.transpose(np.matrix(dvarsNow[0:3])))) 
#            # can add other base configs later
#            kin.base_frame = outputTransform
#         
#            
#        #Brackets                            
#        elif module_types[i] == 'BracketModuleOutside': # left-outside
#            kin.add_bracket('X5-HeavyBracket', 'left-outside')
#            joint_mask.append(0)
#
#        elif module_types[i] == 'BracketModuleInside': # left-outside
#            kin.add_bracket('X5-HeavyBracket', 'left-inside')
#            joint_mask.append(0)
#
#        elif module_types[i] == 'RightLightBracket': # right light bracket
#            kin.add_bracket('X5-LightBracket', 'right')
#            joint_mask.append(0)
#        
#        
#        #Rigid Tube Modules
#        elif module_types[i] == 'InLineTubeModule':
#            #             c= cos(pi/2); s = sin(pi/2);
#            #             dx = dvarsNow(1);
#            #             dy = 0;
#            #             dz = 0.015875;
#            #             output = [c,0,s,dx;  0,1,0,dy; -s,0, c,dz; 0,0, 0, 1];
#            outputTransform = TRp(R_y(pi/2), np.matrix([dvarsNow[0], 0, \
#            0.02])) # 20 mm
#
#            com = np.eye(4)
#            com[0][3] = dvarsNow[0]/2
#            mass= .4*dvarsNow[0] + 0.248
#            kin.add_rigid_body(np.transpose(np.matrix(com[0:3,3])), \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)
#
#
#        elif module_types[i] == 'InLineInputModule':
#            #             c= cos(-pi/2); s = sin(-pi/2);
#            #             dx = -0.015875;
#            #             dy = 0;
#            #             dz = dvarsNow(1);
#            #             output = [c,0,s,dx;  0,1,0,dy; -s,0, c,dz; 0,0, 0, 1];
#            outputTransform = TRp(R_y(-pi/2), np.matrix([-0.02, 0, \
#            dvarsNow[0]])) # 20 mm
#
#            com = np.eye(4)
#            com[2,3] = dvarsNow[0]/2
#            mass= .4*dvarsNow[0] + 0.248
#            kin.add_rigid_body(np.transpose(np.matrix(com[0:3,3])), \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)
        
        
        #Joint Module
        elif module_types[i] == 'RotJointModule': #Make sure that this is actually an actuator
            kin.add_actuator('X5-9')
            joint_mask.append(1)
        
        #Link Modules    
        elif module_types[i] == 'LinkModule':
            kin.add_link('X5', dvarsNow[0], dvarsNow[1]%(2*pi))
            # kin.addBody('X5-Link', 'extension', dvarsNow(1)-0.085, 'twist', 
            # mod(dvarsNow(2),2*pi));
            # disp(['X5-Link', ' extension ', num2str(dvarsNow(1)),  ' twist ', 
            # num2str(mod(dvarsNow(2),2*pi))])
            joint_mask.append(0)

#        elif module_types[i] == 'GenericLink':
#            outputTransform =\
#            TRp(R_z(dvarsNow[5])*R_y(dvarsNow[4])*\
#            R_x(dvarsNow[3]),np.array(np.transpose(np.matrix(dvarsNow[0:3]))))
#            mass = .4*np.linalg.norm(dvarsNow[0:3]) + 0.248 
#            # actual mass per L, add on end caps
#            com = TRp(np.eye(3), np.array(np.transpose(np.matrix(dvarsNow[0:3])))/2)
#            kin.add_rigid_body(com, \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)
#        
#        #Rigid Body End Effectors
#        elif module_types[i] == 'EndEffectorModule':
#            outputTransform = TRp(np.eye(3), np.matrix([0, 0, 0.15])) # 5 mm ?
#            mass = .25
#            com = TRp(np.eye(3), np.array([0, 0, [0.15]])/2)
#            kin.add_rigid_body(com, \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)
#
#        elif module_types[i] == 'EndEffectorVarLModule':
#            outputTransform = TRp(R_y(pi/2), np.array([0, 0, dvarsNow[0]])) 
#            # for the hebi kinematics object, changed  the output frame directions
#            # so that I can use the TipAxis objective
#            mass = .25
#            com = TRp(np.eye(3), np.array([0, 0, dvarsNow[0]])/2)
#            kin.add_rigid_body(com, \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)
#
#
#        elif module_types[i] == 'EndEffectorVarLTModule':
#            outputTransform = TRp(R_z(dvarsNow[1]), np.matrix([0, 0,\
#            dvarsNow[0]]))
#            mass = .25
#            com = TRp(np.eye(3),  np.array([0, 0, dvarsNow[0]])/2)
#            kin.add_rigid_body(com, \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)
#
#        elif module_types[i] == 'EndEffectorVarLTXModule':
#            outputTransform = TRp(R_x(dvarsNow[1]), np.matrix([dvarsNow[0], \
#            0, 0]))
#            mass = .25
#            com = TRp(np.eye(3),  np.array([dvarsNow[0], 0, 0])/2)
#            kin.add_rigid_body(com, \
#            [0,0,0,0,0,0], mass, outputTransform, False)
#            joint_mask.append(0)

        elif module_types[i] == 'EndEffectorVarLXModule':
            outputTransform = TRp(np.eye(3), np.matrix([dvarsNow[0], 0., 0.]))
            mass = .3
            com = TRp(np.eye(3),  np.matrix([dvarsNow[0], 0., 0.])/2)
            kin.add_rigid_body(com, \
            [0.,0.,0.,0.,0.,.0], mass, outputTransform, False)
            joint_mask.append(0)

        #No modules listed
        else:
            print(['Missing module name ', module_types[i]])
#        currentDVar = currentDVar + DVars[i]
    
    return kin, joint_mask


# calculate grav comp torques for the robot
def getGravCompTorques(kin, joint_mask, angles, gravity):
    tauG = []
    if kin.dof_count>0:
        fkcom = kin.get_forward_kinematics('CoM', angles )
        fkout = kin.get_forward_kinematics('output', angles )
        m = kin.masses
#            g = np.matrix([0,0,-9.81]).transpose()
        # joint axes: Z axis of any dof
        for i in range(len(fkout)):
            if (joint_mask[i]==1):  
                    
                # module list starts with Base which is not counted by hebikinematics
                jointAxis = fkout[i][0:3, 2]
                jointPosition = fkout[i][0:3, 3]
#                print('p:' + str(jointPosition.T) + ' ax:'+ str(jointAxis.T)) # for debug
                tauG.append(0)
                for j in range(i+1,len(fkout)):
                    
                    r_comWrtJoint = fkcom[j][0:3, 3] - jointPosition
                    
#                    print('r:' + str(r_comWrtJoint.T) + ' mg:'+ str(m[j]*gravity.T)) # for debug

                    moment = np.cross(r_comWrtJoint, m[j]*gravity, axis=0) # moment = r cross F
        
                    tau_temp = np.inner(jointAxis.T, np.matrix(moment).T) # torques from moment
                    tauG[-1] += tau_temp.item(0)  # extract from matrix
    else:
       tauG = []
       fkout = []
    return tauG, fkout


#if __name__== "__main__":
#
#    kin, joint_mask = make_backpack_kin()
#    gravity= np.matrix([0,0,9.81]).T
#    angles = np.zeros(kin.dof_count)
#    tauG, fkout = getGravCompTorques(kin, joint_mask, angles, gravity)
#    print tauG
#    
#    # optional: plot with this little utility 
#    import robot_plotter
#    robot_plotter.full_plot([], [fkout], str(angles))
    