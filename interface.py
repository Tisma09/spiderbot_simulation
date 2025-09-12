import pybullet as p
import numpy as np

class Interface:

    def __init__(self, client_id):
        self.world_parent_id = client_id

        self.current_mode = 0 
        """ 1 : Mode Auto
            2 : Mode Manuel
            3 : Mode ..."""
        
        self.auto_button_id = p.addUserDebugParameter(" Mode Auto", 1, 0, 1, physicsClientId=client_id)
        self.manual_button_id = p.addUserDebugParameter(" Mode Manuel", 1, 0, 1, physicsClientId=client_id)

        self.param_AD_H = None
        self.param_AD_V = None

        self.param_AG_H = None
        self.param_AG_V = None

        self.param_DD_H = None
        self.param_DD_V = None

        self.param_DG_H = None
        self.param_DG_V = None

        
    def init_auto(self):
        self.current_mode = 1

        self.param_x = p.addUserDebugParameter(" X Goal Position", -100.0, 200.0, 0.0, physicsClientId=self.world_parent_id)
        self.param_y = p.addUserDebugParameter(" Y Goal Position", 0.0, 200.0, 94.5, physicsClientId=self.world_parent_id)
        self.param_z = p.addUserDebugParameter(" Z Position", -100.0, 0.0, -97.64, physicsClientId=self.world_parent_id)

    def end_auto(self):
        self.current_mode = 0

        p.removeAllUserParameters(physicsClientId=self.world_parent_id)
        self.auto_button_id = p.addUserDebugParameter(" Mode Auto", 1, 0, 1, physicsClientId=self.world_parent_id)
        self.manual_button_id = p.addUserDebugParameter(" Mode Manuel", 1, 0, 1, physicsClientId=self.world_parent_id)


    def init_manual(self):
        self.current_mode = 2

        self.param_ARD_H = p.addUserDebugParameter(" Joint ARD_H Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_ARD_V1 = p.addUserDebugParameter(" Joint ARD_V1 Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_ARD_V2 = p.addUserDebugParameter(" Joint ARD_V2 Position", -2.35619, 0.785398, 0.0, physicsClientId=self.world_parent_id)
        self.param_AVG_H = p.addUserDebugParameter(" Joint AVG_H Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_AVG_V1 = p.addUserDebugParameter(" Joint AVG_V1 Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_AVG_V2 = p.addUserDebugParameter(" Joint AVG_V2 Position", -2.35619, 0.785398, 0.0, physicsClientId=self.world_parent_id)
        self.param_AVD_H = p.addUserDebugParameter(" Joint AVD_H Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_AVD_V1 = p.addUserDebugParameter(" Joint AVD_V1 Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_AVD_V2 = p.addUserDebugParameter(" Joint AVD_V2 Position", -2.35619, 0.785398, 0.0, physicsClientId=self.world_parent_id)
        self.param_ARG_H = p.addUserDebugParameter(" Joint ARG_H Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_ARG_V1 = p.addUserDebugParameter(" Joint ARG_V1 Position", -1.570796, 1.570796, 0.0, physicsClientId=self.world_parent_id)
        self.param_ARG_V2 = p.addUserDebugParameter(" Joint ARG_V2 Position", -2.35619, 0.785398, 0.0, physicsClientId=self.world_parent_id)

    def end_manual(self):
        self.current_mode = 0

        p.removeAllUserParameters(physicsClientId=self.world_parent_id)
        self.auto_button_id = p.addUserDebugParameter(" Mode Auto", 1, 0, 1, physicsClientId=self.world_parent_id)
        self.manual_button_id = p.addUserDebugParameter(" Mode Manuel", 1, 0, 1, physicsClientId=self.world_parent_id)
