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

        # Mode
        self.run = False
        self.turn = False
        self.turn_last_cycle = False
        self.t0_rot = 0
        self.angle_to_turn = 0


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
        # Cmd 
        self.run = p.readUserDebugParameter(interface.run_id) % 2 == 0 
        self.turn = p.readUserDebugParameter(interface.rot_id) % 2 == 0 
        angle_target = p.readUserDebugParameter(interface.angle_id, physicsClientId=interface.world_parent_id)


        # Paramètres
        step_length = 120
        step_height = 70
        step_height_rot = 30
        T = 0.25 
        T_rot = 0.5

        # Phases Marche
        phase_AVD_ARG = t            # AVD + ARG en phase
        phase_AVG_ARD = t + T/2.0    # AVG + ARD décalées 

        # Position origine
        x_std, y_std, z_std = 80, 80, -80


        # Mode turn
        dx, dy, dz = 0, 0, 0 
        x_leg_offset_std = 38.5 
        y_leg_offset_std = 27.175 
        if self.turn :
            if not self.turn_last_cycle :
                self.t0_rot=t
                self.angle_to_turn = angle_target
            t_rot=t-self.t0_rot
            angle_rot_past = min(self.angle_to_turn, np.pi/8) if angle_target > 0 else max(self.angle_to_turn, -np.pi/8)
            z_std = -60
            if t_rot >= T_rot :
                self.angle_to_turn = self.angle_to_turn - angle_rot_past
                if abs(self.angle_to_turn) > 0.001:
                    self.t0_rot=t
            angle_rot = min(self.angle_to_turn, np.pi/8) if angle_target > 0 else max(self.angle_to_turn, -np.pi/8)



        # Deplacement
        # AVD
        x, y, z = x_std, y_std, z_std
        if self.run :
            dx, dy, dz = foot_traj(phase_AVD_ARG, step_length, step_height, T)
            z_eq = 20
            x, y, z = x+dx, y+dy, z+z_eq+dz
        if self.turn :
            x_start_in_bot = x + x_leg_offset_std
            y_start_in_bot = y + y_leg_offset_std
            x, y, z = rot_traj(t_rot, 1, (x_start_in_bot, y_start_in_bot, z), angle_rot, step_height_rot, T_rot)
        self.q[9], self.q[10], self.q[11] = ik_leg((x, y, z), leg_rotation=0.0)
        # ARG
        x, y, z = -x_std, -y_std, z_std
        if self.run :
            dx, dy, dz = foot_traj(phase_AVD_ARG, step_length, step_height, T)
            z_eq = -30
            x, y, z = x+dx, y+dy, z+z_eq+dz
        if self.turn :
            x_start_in_bot = x - x_leg_offset_std
            y_start_in_bot = y - y_leg_offset_std
            x, y, z = rot_traj(t_rot, 2, (x_start_in_bot, y_start_in_bot, z), angle_rot, step_height_rot, T_rot)
        self.q[13], self.q[14], self.q[15] = ik_leg((x, y, z), leg_rotation=np.pi)

        
        # AVG 
        x, y, z = x_std, -y_std, z_std
        if self.run :
            dx, dy, dz = foot_traj(phase_AVG_ARD, step_length, step_height, T)
            z_eq = 20
            x, y, z = x+dx, y+dy, z+z_eq+dz
        if self.turn :
            x_start_in_bot = x + x_leg_offset_std
            y_start_in_bot = y - y_leg_offset_std
            x, y, z = rot_traj(t_rot, 3, (x_start_in_bot, y_start_in_bot, z), angle_rot, step_height_rot, T_rot)
        self.q[5], self.q[6], self.q[7] = ik_leg((-x, y, z), leg_rotation=np.pi)
        # ARD
        x, y, z = -x_std, y_std, z_std
        if self.run :
            dx, dy, dz = foot_traj(phase_AVG_ARD, step_length, step_height, T)
            z_eq = -30
            x, y, z = x+dx, y+dy, z_std+z_eq+dz
        if self.turn :
            x_start_in_bot = x - x_leg_offset_std
            y_start_in_bot = y + y_leg_offset_std
            x, y, z = rot_traj(t_rot, 4, (x_start_in_bot, y_start_in_bot, z), angle_rot, step_height_rot, T_rot)
        self.q[1], self.q[2], self.q[3] = ik_leg((-x, y, z), leg_rotation=0.0)

        
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


def rot_traj(t, phase_id, xyz_start_in_bot, angle, step_height, T=2.0):
    """
    Trajectoire d'un pied pour rotation.
    """
    x_start_in_bot, y_start_in_bot, z_start_in_bot = xyz_start_in_bot
    x_leg_offset = 38.5 if phase_id == 1 or phase_id == 3 else -38.5
    y_leg_offset = 27.175 if phase_id == 1 or phase_id == 4 else -27.175 
    phase = min(max(t / T, 0.0), 1.0)
    start = (phase_id-1) / 4
    end   = phase_id / 4

    if phase < start:
        x = x_start_in_bot - x_leg_offset
        y = y_start_in_bot - y_leg_offset
        z = z_start_in_bot
    elif start <= phase < end:
        local_phase = (phase - start) / (end - start)
        x = x_start_in_bot * math.cos(angle * local_phase) - y_start_in_bot * math.sin(angle * local_phase) - x_leg_offset
        y = x_start_in_bot * math.sin(angle * local_phase) + y_start_in_bot * math.cos(angle * local_phase) - y_leg_offset
        z = step_height * math.sin(local_phase * math.pi)
    elif end <= phase :
        x = x_start_in_bot * math.cos(angle) - y_start_in_bot * math.sin(angle) - x_leg_offset
        y = x_start_in_bot * math.sin(angle) + y_start_in_bot * math.cos(angle) - y_leg_offset
        z = z_start_in_bot

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
