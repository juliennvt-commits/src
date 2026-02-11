#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time
import threading
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2

def main():
    rclpy.init()
    
    node = Node(
        "ur3e_stacker_final",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    
    # Utilisation du SingleThreadedExecutor pour la stabilité
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("--- DÉMARRAGE DU SCRIPT ---")
    print("Vérification de la connexion Gazebo/MoveIt... (5s)")
    time.sleep(5.0) 

    arm = MoveIt2(
        node=node,
        joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur3e",
    )
    
    arm.max_velocity_scaling_factor = 0.1
    arm.max_acceleration_scaling_factor = 0.1
    arm.planning_time = 15.0 

    # --- TES DONNÉES DE POSITIONS ---
    robot_X, robot_Y, robot_Z = 0.4, -0.68, 0.89
    cube_world_X, cube_world_Y, cube_world_Z = 0.3, -0.3, 0.91 
    
    b1_world_X, b1_world_Y = 0.13, -0.375
    b2_world_X, b2_world_Y = 0.35, -0.105
    b3_world_X, b3_world_Y = 0.615, -0.18

    SIZE_B1 = [0.16, 0.55, 0.07]
    SIZE_B2 = [0.26, 0.11, 0.11]
    SIZE_B3 = [0.27, 0.26, 0.15]
    SIZE_TABLE = [1.5, 1.2, 0.001]

    # CALCUL DES POSITIONS RELATIVES
    TARGET_X = cube_world_X - robot_X   
    TARGET_Y = cube_world_Y - robot_Y   
    TARGET_Z = cube_world_Z - robot_Z   

    T1_X, T1_Y, T1_Z = b1_world_X - robot_X, b1_world_Y - robot_Y, 0.07 / 2
    T2_X, T2_Y, T2_Z = b2_world_X - robot_X, b2_world_Y - robot_Y, 0.11 / 2
    T3_X, T3_Y, T3_Z = b3_world_X - robot_X, b3_world_Y - robot_Y, 0.15 / 2

    CUBE_NAME = "cube_rouge"
    GRIPPER_OFFSET = 0.18 

    # --- MISE EN PLACE DE LA SCÈNE ---
    print("Nettoyage et ajout des obstacles...")
    for obj in [CUBE_NAME, "box1", "box2", "box3", "table_plan"]:
        arm.remove_collision_object(obj)
    time.sleep(1.0)

    arm.add_collision_box("table_plan", SIZE_TABLE, "box", [0.0, 0.0, -0.02], [0.0, 0.0, 0.0, 1.0])
    arm.add_collision_box("box1", SIZE_B1, "box", [T1_X, T1_Y, T1_Z], [0.0, 0.0, 0.0, 1.0])
    arm.add_collision_box("box2", SIZE_B2, "box", [T2_X, T2_Y, T2_Z], [0.0, 0.0, 0.0, 1.0])
    arm.add_collision_box("box3", SIZE_B3, "box", [T3_X, T3_Y, T3_Z], [0.0, 0.0, 0.0, 1.0])
    arm.add_collision_box(CUBE_NAME, [0.06, 0.06, 0.06], "box", [TARGET_X, TARGET_Y, TARGET_Z], [0.0, 0.0, 0.0, 1.0])
    time.sleep(2.0)

    # --- DÉROULÉ DU TEST ---

    # ÉTAPE 0 : Home
    print("--- ÉTAPE 0 : Home ---")
    arm.move_to_configuration([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    arm.wait_until_executed()

    # ÉTAPE A : Approche
    print(f"--- ÉTAPE A : Approche vers X:{TARGET_X:.2f}, Y:{TARGET_Y:.2f} ---")
    
    pose_target = Pose()
    pose_target.position.x = TARGET_X
    pose_target.position.y = TARGET_Y
    pose_target.position.z = TARGET_Z + GRIPPER_OFFSET
    
    pose_target.orientation.x = 1.0 # Pince vers le bas
    pose_target.orientation.y = 0.0
    pose_target.orientation.z = 0.0
    pose_target.orientation.w = 0.0

    success = arm.move_to_pose(pose_target)
    
    if success:
        arm.wait_until_executed()
        print(">>> SUCCÈS : Arrivé au dessus du cube !")
    else:
        time.sleep(2.0)
        print(">>> Note : Vérifie Gazebo, le robot a pu bouger malgré l'absence de retour.")

    print("Fin du script.")
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
