import pybullet as p
import numpy as np
import math

from interface import Interface
from kinematics import *

class Robot:

    def __init__(self, client_id, path, robot_start_pos, robot_start_orientation):

        self.world_parent_id = client_id
        self.id = p.loadURDF(path, 
                            robot_start_pos, 
                            robot_start_orientation,
                            flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                            physicsClientId=client_id)
        self.num_joints = p.getNumJoints(self.id, physicsClientId=client_id)
        self.q = np.zeros(self.num_joints)


    ############################################################################################
    #########################               Manual Mode              ###########################
    ############################################################################################

    def manual_move(self, interface) :
        """
        Mouvement en mode manuel puis vérification de collision
        Parameters: 
            debug_parameters
        Returns:
        """
        self.q[1] = p.readUserDebugParameter(interface.param_ARD_H, physicsClientId=interface.world_parent_id)
        self.q[2] = p.readUserDebugParameter(interface.param_ARD_V1, physicsClientId=interface.world_parent_id)
        self.q[3] = p.readUserDebugParameter(interface.param_ARD_V2, physicsClientId=interface.world_parent_id)
        self.q[5] = p.readUserDebugParameter(interface.param_AVG_H, physicsClientId=interface.world_parent_id)
        self.q[6] = p.readUserDebugParameter(interface.param_AVG_V1, physicsClientId=interface.world_parent_id)
        self.q[7] = p.readUserDebugParameter(interface.param_AVG_V2, physicsClientId=interface.world_parent_id)
        self.q[9] = p.readUserDebugParameter(interface.param_AVD_H, physicsClientId=interface.world_parent_id)
        self.q[10] = p.readUserDebugParameter(interface.param_AVD_V1, physicsClientId=interface.world_parent_id)
        self.q[11] = p.readUserDebugParameter(interface.param_AVD_V2, physicsClientId=interface.world_parent_id)
        self.q[13] = p.readUserDebugParameter(interface.param_ARG_H, physicsClientId=interface.world_parent_id)
        self.q[14] = p.readUserDebugParameter(interface.param_ARG_V1, physicsClientId=interface.world_parent_id)
        self.q[15] = p.readUserDebugParameter(interface.param_ARG_V2, physicsClientId=interface.world_parent_id)

    ############################################################################################
    #########################             Autonomous Mode            ###########################
    ############################################################################################

    def init_auto_pos(self):
        pass



    def autonomous_move(self, interface, t):
        x = p.readUserDebugParameter(interface.param_x, physicsClientId=interface.world_parent_id)
        y = p.readUserDebugParameter(interface.param_y, physicsClientId=interface.world_parent_id)
        z = p.readUserDebugParameter(interface.param_z, physicsClientId=interface.world_parent_id)

        result = ik_leg((x, y, z))
        if result is not None:
            q1_result, q2_result, q3_result = result

            self.q[9] = q1_result
            self.q[10] = q2_result 
            self.q[11] = q3_result









    ############################################################################################
    #########################              Debug Fct                 ###########################
    ############################################################################################


    def update_joint_axes(self, joint_line_ids):
        for i in range(p.getNumJoints(self.id)):
            joint_info = p.getJointInfo(self.id, i)
            joint_type = joint_info[2]
            
            # Filtre pour joints rotatifs/glissières uniquement
            if joint_type not in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                continue
                
            # Récupération infos parent
            parent_id = joint_info[16]
            if parent_id == -1:
                parent_pos, parent_orn = p.getBasePositionAndOrientation(self.id)
            else:
                parent_state = p.getLinkState(self.id, parent_id)
                parent_pos = np.array(parent_state[0])
                parent_orn = parent_state[1]
            
            # Calcul position/orientation du joint
            parent_frame_pos = np.array(joint_info[14])
            rot_matrix = np.array(p.getMatrixFromQuaternion(parent_orn)).reshape(3, 3)
            joint_pos = parent_pos + rot_matrix.dot(parent_frame_pos)
            
            # Calcul direction de l'axe dans l'espace global
            axis_local = np.array(joint_info[13])
            axis_global = rot_matrix.dot(axis_local)
            axis_global = axis_global / np.linalg.norm(axis_global) * 0.2
            end_pos = joint_pos + axis_global
            
            # Mise à jour des lignes
            if i in joint_line_ids:
                p.addUserDebugLine(joint_pos.tolist(), end_pos.tolist(), 
                                [1, 0.5, 0], replaceItemUniqueId=joint_line_ids[i], lineWidth=3)
            else:
                joint_line_ids[i] = p.addUserDebugLine(joint_pos.tolist(), end_pos.tolist(),
                                                    [1, 0.5, 0], lineWidth=3)
