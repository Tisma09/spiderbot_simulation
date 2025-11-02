import pybullet as p
import pybullet_data
import numpy as np

from robot import Robot
from interface import Interface

class WorldSimulate:

    def __init__(self, client_type, env_path, robot_path):
        self.client_type = client_type
        self.client_id = p.connect(client_type)

        #self.ground_id = self.load_world(env_path)
        self.ground_id = self.create_heightfield_ground()
        self.bot = self.load_object(robot_path)

        self.interface = Interface(self.client_id) if client_type == p.GUI else None


    def load_world(self, env_path):
        """
        Load env_path
        Parameters: 
            env_path (str) : URDF file of env_path
        Returns: 
            ground_id (int) : Index of the PyBullet object
        """
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)

        ground_id = p.loadURDF(env_path, physicsClientId=self.client_id)
        
        return ground_id


    def create_heightfield_ground(self):
        """
        Create a heightfield ground with three zones: left incline, flat middle, right incline
        """
        p.setGravity(0, 0, -9.81)
        
        # Dimensions 
        numRows = 64
        numCols = 64
        heightfieldData = np.zeros((numRows, numCols))

        # Zone
        left_cols = numCols // 4
        middle_cols = numCols // 2
        right_cols = numCols - (left_cols + middle_cols)
        bump_amplitude = 0.01

        # Zone left : Up
        for j in range(left_cols):
            heightfieldData[:, j] = 0.01 * j + np.random.uniform(-bump_amplitude, bump_amplitude, size=numRows)

        # Zone center : flat
        flat_height = 0.01 * (left_cols - 1)
        heightfieldData[:, left_cols:left_cols+middle_cols] = flat_height

        # Zone right : Up
        for j in range(right_cols):
            start_height = heightfieldData[:, left_cols + middle_cols - 1]
            heightfieldData[:, left_cols + middle_cols + j] = start_height + 0.01 * j + np.random.uniform(-bump_amplitude, bump_amplitude, size=numRows)

        # Field
        field_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[0.05, 0.05, 1],
            heightfieldTextureScaling=(numRows - 1)/2,
            heightfieldData=heightfieldData.flatten(),
            numHeightfieldRows=numRows,
            numHeightfieldColumns=numCols
        )

        field = p.createMultiBody(0, field_shape, basePosition=[0,0,0])
        return field






    def load_object(self, robot_path):
        """
        Load robot
        Parameters: 
            robot_path (str) : URDF file of robot
        Returns: 
            robot (Robot) : Object of class Robot
        """
        robot_start_pos = [0, 0, 0.8]  # Augmentation de la hauteur initiale
        robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
        bot = Robot(self.client_id, 
                        robot_path, 
                        robot_start_pos, 
                        robot_start_orientation)
        
        p.changeVisualShape(bot.id, -1, rgbaColor=[0.6, 0, 0, 1], physicsClientId=self.client_id) 
        for i in range(0, 16) :
            if i%2 == 0:
                p.changeVisualShape(bot.id, i, rgbaColor=[0, 0, 0, 1], physicsClientId=self.client_id)  # Noir
            else :
                p.changeVisualShape(bot.id, i, rgbaColor=[0.6, 0, 0, 1], physicsClientId=self.client_id)  # Rouge

            
        return bot
