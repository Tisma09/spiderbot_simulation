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
        self.run = False
        self.turn = False
        self.turn_last_cycle = False
        self.t0_rot = 0


    ############################################################################################
    #########################               Manual Mode              ###########################
    ############################################################################################

    def manual_move(self, interface) :
        """
        Mouvement en mode manuel
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

    def manual_cart_move(self, interface) :
        """
        Mouvement en mode manuel en controle de position cartésienne des pattes
        Parameters: 
            debug_parameters
        Returns:
        """
        # AVD
        x = p.readUserDebugParameter(interface.param_x_AVD, physicsClientId=interface.world_parent_id)
        y = p.readUserDebugParameter(interface.param_y_AVD, physicsClientId=interface.world_parent_id)
        z = p.readUserDebugParameter(interface.param_z_AVD, physicsClientId=interface.world_parent_id)
        result = ik_leg((x, y, z), leg_rotation=0.0)
        if result is not None:
            self.q[9], self.q[10], self.q[11] = result
        # AVG
        x = p.readUserDebugParameter(interface.param_x_AVG, physicsClientId=interface.world_parent_id)
        y = p.readUserDebugParameter(interface.param_y_AVG, physicsClientId=interface.world_parent_id)
        z = p.readUserDebugParameter(interface.param_z_AVG, physicsClientId=interface.world_parent_id)
        result = ik_leg((-x, y, z), leg_rotation=np.pi)
        if result is not None:
            self.q[5], self.q[6], self.q[7] = result
        # ARD
        x = p.readUserDebugParameter(interface.param_x_ARD, physicsClientId=interface.world_parent_id)
        y = p.readUserDebugParameter(interface.param_y_ARD, physicsClientId=interface.world_parent_id)
        z = p.readUserDebugParameter(interface.param_z_ARD, physicsClientId=interface.world_parent_id)
        result = ik_leg((-x, y, z), leg_rotation=0.0)
        if result is not None:
            self.q[1], self.q[2], self.q[3] = result
        # ARG
        x = p.readUserDebugParameter(interface.param_x_ARG, physicsClientId=interface.world_parent_id)
        y = p.readUserDebugParameter(interface.param_y_ARG, physicsClientId=interface.world_parent_id)
        z = p.readUserDebugParameter(interface.param_z_ARG, physicsClientId=interface.world_parent_id)
        result = ik_leg((x, y, z), leg_rotation=np.pi)
        if result is not None:
            self.q[13], self.q[14], self.q[15] = result

    ############################################################################################
    #########################             Autonomous Mode            ###########################
    ############################################################################################

    def init_auto_pos(self):
        pass



    def autonomous_move(self, interface, t):
        # Paramètres
        step_length = 120
        step_height = 70
        step_height_rot = 30
        T = 0.5 
        T_rot = 0.5

        # Position origine
        #x_std, y_std, z_std = 50, 94.5, -80
        x_std, y_std, z_std = 80, 80, -80

        # Phases 
        phase_FR_ARG = t            # AVD + ARG en phase
        phase_FL_ARD = t + T/2.0    # AVG + ARD décalées 

        # Cmd 
        self.run = p.readUserDebugParameter(interface.run_id) % 2 == 0 
        self.turn = p.readUserDebugParameter(interface.rot_id) % 2 == 0 
        if self.turn and not self.turn_last_cycle :
            self.t0_rot=t
        dx, dy, dz = 0, 0, 0

        # Mode 
        if self.turn :
            t_rot=t-self.t0_rot
            z_std = -50
        
        
        if self.run :
            dx, dy, dz = foot_traj(phase_FR_ARG, step_length, step_height, T)
        # AVD
        x, y, z = x_std, y_std, z_std
        if self.turn :
            t_rot=t-self.t0_rot
            dx, dy, dz = rot_traj(t_rot, 1, 3, x, y, np.pi/6, step_height_rot, T_rot)
        z_eq = 20 if self.run else 0
        self.q[9], self.q[10], self.q[11] = ik_leg((x+dx, y+dy, z+z_eq+dz), leg_rotation=0.0)
        # ARG
        x, y, z = -x_std, -y_std, z_std
        if self.turn :
            t_rot=t-self.t0_rot
            dx, dy, dz = rot_traj(t_rot, 3, 1, x, y, np.pi/6, step_height_rot, T_rot)
        z_eq = -30 if self.run else 0
        self.q[13], self.q[14], self.q[15] = ik_leg((x+dx, y+dy, z+z_eq+dz), leg_rotation=np.pi)

        
        if self.run :
            dx, dy, dz = foot_traj(phase_FL_ARD, step_length, step_height, T)
        # AVG 
        x, y, z = 80, -y_std, z_std
        if self.turn :
            t_rot=t-self.t0_rot
            dx, dy, dz = rot_traj(t_rot, 4, 2, x, y, np.pi/6, step_height_rot, T_rot)
        z_eq = 20 if self.run else 0
        self.q[5], self.q[6], self.q[7] = ik_leg((-x-dx, y+dy, z+z_eq+dz), leg_rotation=np.pi)
        # ARD
        x, y, z = -x_std, y_std, z_std
        if self.turn :
            t_rot=t-self.t0_rot
            dx, dy, dz = rot_traj(t_rot, 2, 4, x, y, np.pi/6, step_height_rot, T_rot)
        z_eq = -30 if self.run else 0
        self.q[1], self.q[2], self.q[3] = ik_leg((-x-dx, y+dy, z+z_eq+dz), leg_rotation=0.0)

        
        self.turn_last_cycle = self.turn




def foot_traj(t, step_length=20, step_height=10, T=2.0):
    """
    Trajectoire d'un pied pour le trot.
    t : temps
    step_length : amplitude en X (mm)
    step_height : hauteur de levée (mm)
    T : période du cycle (s)
    """
    phase = (t % T) / T  # phase dans [0,1)

    if phase < 0.5:  
        # Au sol
        x = - step_length * (phase - 0.25)
        z = 0
    else:
        # En l'air
        x = step_length * (phase - 0.75)
        z = step_height * math.sin((phase - 0.5) * math.pi * 2)

    return (x, 0, z)


def rot_traj(t, phase_id, phase_comp, x_start, y_start, angle, step_height, T=2.0):
    """
    Trajectoire d'un pied pour rotation.
    """
    phase = min(max(t / T, 0.0), 1.0)
    start = (phase_id-1) / 4
    end   = phase_id / 4
    start_comp = (phase_comp-1) / 4
    end_comp = phase_comp / 4

    if start <= phase < end:
        local_phase = (phase - start) / (end - start)

        x = x_start * math.cos(angle * local_phase) - y_start * math.sin(angle * local_phase)
        y = x_start * math.sin(angle * local_phase) + y_start * math.cos(angle * local_phase)
        x -= x_start
        y -= y_start
        z = step_height * math.sin(local_phase * math.pi)
    elif start_comp <= phase < end_comp:
        local_phase = (phase - start_comp) / (end_comp - start_comp)
        z = 10 * math.sin(local_phase * math.pi)
    else :
        z = 0

    if phase < start :
        x = 0
        y = 0
    if phase >= end:
        x = x_start * math.cos(angle) - y_start * math.sin(angle) - x_start
        y = x_start * math.sin(angle) + y_start * math.cos(angle) - y_start

    return (x, y, z)



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
