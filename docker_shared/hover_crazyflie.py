import os
import time
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

FILENAME = "mesh.obj" 
DIRECTORY = "/shared"
OBJ_PATH = os.path.join(DIRECTORY, FILENAME)

MESH_SCALE = [1.0, 1.0, 1.0]
# Translation: [X, Y, Z]. 
# Adjust Z to ensure the visual floor is slightly below the physics floor (z=0)
MESH_TRANSLATION = [0.0, 0.0, -0.05] 
# Rotation: [Roll, Pitch, Yaw] in DEGREES
MESH_ROTATION_EULER = [90, 0, 0] 

HOVER_DURATION = 6         
SIM_FREQ = 240
CTRL_FREQ = 48

def run_simulation():
    if not os.path.exists(OBJ_PATH):
        print(f"\n[ERROR] File not found: {OBJ_PATH}")
        return

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0, 0, 0.2]]), 
        physics=Physics.PYB,
        gui=True,
        pyb_freq=SIM_FREQ,
        ctrl_freq=CTRL_FREQ
    )

    obs, info = env.reset()
    
    client_id = env.getPyBulletClient()
    p.setAdditionalSearchPath(DIRECTORY, physicsClientId=client_id)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=client_id)
    p.changeVisualShape(0, -1, rgbaColor=[0, 0, 0, 0], physicsClientId=client_id)

    abs_path = os.path.abspath(OBJ_PATH)
    
    rotation_quat = p.getQuaternionFromEuler([
        np.radians(MESH_ROTATION_EULER[0]),
        np.radians(MESH_ROTATION_EULER[1]),
        np.radians(MESH_ROTATION_EULER[2])
    ])

    visual_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=abs_path,
        meshScale=MESH_SCALE,
        physicsClientId=client_id,
        rgbaColor=[0.8, 0.8, 0.8, 1] 
    )
    
    room_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1, 
        baseVisualShapeIndex=visual_id,
        basePosition=MESH_TRANSLATION,
        baseOrientation=rotation_quat,
        physicsClientId=client_id
    )
    
    print(f"Mesh injected at {MESH_TRANSLATION} with rotation {MESH_ROTATION_EULER}")

    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
    
    start_time = time.time()
    actual_timestep = 1.0 / SIM_FREQ
    
    for i in range(15 * SIM_FREQ):
        elapsed = i / SIM_FREQ
        
        target_pos = np.array([0, 0, 1.0]) if elapsed < HOVER_DURATION else np.array([0, 0, 0.03])

        state = env._getDroneStateVector(0)
        
        action, _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP, 
            state=state,
            target_pos=target_pos,
            target_rpy=np.array([0, 0, 0])
        )

        action_array = np.reshape(action, (1, 4))
        obs, reward, terminated, truncated, info = env.step(action_array)
        
        if i % 10 == 0:
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1, physicsClientId=client_id)

        sync(i, start_time, actual_timestep)
        
        if terminated or truncated:
            break

    env.close()

if __name__ == "__main__":
    run_simulation()