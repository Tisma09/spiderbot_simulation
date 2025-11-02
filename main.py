import time
import keyboard
import pybullet as p
import numpy as np

from world_simulate import WorldSimulate 



################################################################################################
#########################               Main                    ################################
################################################################################################

debug = False
#physics_world = WorldSimulate(p.GUI, "plane.urdf", "Robot_mesh_urdf_V2/RobotProto_With_Col.urdf")
physics_world = WorldSimulate(p.GUI, "plane.urdf", "Robot_mesh_urdf_V3/RobotSpider_With_Col.urdf")

p.resetDebugVisualizerCamera(
    cameraDistance=0.4,             # Zoom (augmenter pour dézoomer)
    cameraYaw=50,                   # Angle de rotation (0 = face avant)
    cameraPitch=-35,                # Inclinaison vers le bas (-30 pour une vue plongeante)
    cameraTargetPosition=[0, 0, 0], # Point que la caméra vise (centre du robot)
    physicsClientId=physics_world.client_id
)

if debug :
    p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
    joint_line_ids = {} 

T = 1.0 / 240.0
t= 0
while True:
    start_time = time.time()

    if debug:
        physics_world.bot.update_joint_axes(joint_line_ids)


    # Mode auto
    if p.readUserDebugParameter(physics_world.interface.auto_button_id) % 2 == 0 and physics_world.interface.current_mode == 0 :
        print("Start Auto mode")
        physics_world.interface.init_auto()
        physics_world.bot.init_auto_pos()
    elif p.readUserDebugParameter(physics_world.interface.auto_button_id) % 2 == 1 and physics_world.interface.current_mode == 1 :
        print("End Auto mode")
        physics_world.interface.end_mode()
    # Mode manuel
    elif p.readUserDebugParameter(physics_world.interface.manual_button_id) % 2 == 0 and physics_world.interface.current_mode == 0 :
        print("Start Manual mode")
        physics_world.interface.init_manual()
    elif p.readUserDebugParameter(physics_world.interface.manual_button_id) % 2 == 1 and physics_world.interface.current_mode == 2 :
        print("End Manual mode")
        physics_world.interface.end_mode()
    # Mode manuel cartesien
    elif p.readUserDebugParameter(physics_world.interface.manual_cart_button_id) % 2 == 0 and physics_world.interface.current_mode == 0 :
        print("Start Manual Cartesian mode")
        physics_world.interface.init_manual_cart()
    elif p.readUserDebugParameter(physics_world.interface.manual_cart_button_id) % 2 == 1 and physics_world.interface.current_mode == 3 :
        print("End Manual Cartesian mode")
        physics_world.interface.end_mode()
    

    # Move mode
    if physics_world.interface.current_mode == 1 :
        physics_world.bot.autonomous_move(physics_world.interface, t)
            
    if physics_world.interface.current_mode == 2 :
        physics_world.bot.manual_move(physics_world.interface)

    if physics_world.interface.current_mode == 3 :
        physics_world.bot.manual_cart_move(physics_world.interface)
    
    # Mise à jour des positions des moteurs
    for joint_index, position in enumerate(physics_world.bot.q):
        if joint_index < physics_world.bot.num_joints: 
            p.setJointMotorControl2(
                bodyIndex=physics_world.bot.id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=position,
                force=100,
                physicsClientId = physics_world.client_id
            )


    # Avance simulation
    p.stepSimulation(physicsClientId=physics_world.client_id)

    # Calculs temps
    elapsed_time = time.time() - start_time
    sleep_time = max(0, T - elapsed_time)
    time.sleep(sleep_time)
    t += sleep_time

    if keyboard.is_pressed('q'): 
        print("Simulation terminée.")
        break


p.disconnect()