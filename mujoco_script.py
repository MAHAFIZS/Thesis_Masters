import time
import mujoco
import numpy as np
import mujoco.viewer
import os
import socket
import struct
# === Config ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_ip = "127.0.0.1"
port = 5005
print("🚀 Python script started.")

def send_gesture(gesture_id):
    sock.sendto(str(gesture_id).encode(), (server_ip, port))
model_path = "D:\\mujoco_menagerie\\franka_emika_panda\\mjx_single_cube.xml"

STEP_SIZE = 0.01
ANGLE_STEP = np.deg2rad(5)

# === Load Model ===
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# === Get body ID for gripper ===
GRIPPER_BODY_ID = model.body("hand").id

finger1_idx = model.actuator("finger1_act").id
finger2_idx = model.actuator("finger2_act").id
finger1_qpos_adr = model.joint("finger_joint1").qposadr
finger2_qpos_adr = model.joint("finger_joint2").qposadr



# === Gesture reader ===

# === Helper functions ===
def get_gripper_position():
    return data.body(GRIPPER_BODY_ID).xpos.copy()

def clamp_joint_positions(qpos):
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        min_val, max_val = model.jnt_range[j]
        if max_val > min_val:
            qpos[j] = np.clip(qpos[j], min_val, max_val)

def move_cartesian_custom(dx=0, dy=0, dz=0):
    print(f"🚀 move_cartesian_custom called with dx={dx}, dy={dy}, dz={dz}")

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, get_gripper_position(), GRIPPER_BODY_ID)

    delta = np.array([dx, dy, dz])
    J = jacp
    dq = np.linalg.pinv(J).dot(delta)

    dq_full = np.zeros_like(data.qpos)
    for i in [1, 3, 5]:  # actuator1,3,5 for Y rotation
        joint_id = model.actuator_trnid[i][0]
        dq_full[joint_id] = dq[joint_id]
    for i in [2, 4]:  # actuator2,4 for X/Z
        joint_id = model.actuator_trnid[i][0]
        dq_full[joint_id] = dq[joint_id]
    if dx != 0 or dz != 0:
        joint_id = model.actuator_trnid[6][0]  # actuator6 (move away from body)
        dq_full[joint_id] = dq[joint_id]

    # Apply joint updates
    data.qpos[:7] += dq_full[:7]
    clamp_joint_positions(data.qpos)
    mujoco.mj_forward(model, data)

    # Apply to actuators
    for i in range(7):
        jnt_id = model.actuator_trnid[i][0]
        data.ctrl[i] = data.qpos[jnt_id]

    mujoco.mj_step(model, data)
    print(f"✅ Applied motion. New qpos[:7]: {np.round(data.qpos[:7], 4)}")


def rotate_gripper(step=0.05):
    try:
        joint_id = model.joint("joint7").id
        act_id = model.actuator("actuator7").id
        data.qpos[joint_id] = np.clip(data.qpos[joint_id] + step, -2.8, 2.8)
        mujoco.mj_forward(model, data)
        data.ctrl[act_id] = data.qpos[joint_id]
        mujoco.mj_step(model, data)
        print(f"✅ Rotated: joint7 = {data.qpos[joint_id]:.4f}")
    except Exception as e:
        print(f"❌ Failed to rotate gripper: {e}")

def open_gripper(step=0.002):
    try:
        a1 = model.actuator("finger1_act").id
        a2 = model.actuator("finger2_act").id
        j1 = model.joint("finger_joint1").qposadr
        j2 = model.joint("finger_joint2").qposadr

        # Increase both equally
        val = np.clip(data.qpos[j1] + step, 0.0, 0.04)
        data.ctrl[a1] = val
        data.ctrl[a2] = val
        print(f"✅ Gripper opening: {float(val):.4f}")

    except Exception as e:
        print(f"❌ Failed to open gripper: {e}")


def close_gripper(step=0.002):
    try:
        a1 = model.actuator("finger1_act").id
        a2 = model.actuator("finger2_act").id
        j1 = model.joint("finger_joint1").qposadr
        j2 = model.joint("finger_joint2").qposadr

        # Decrease both equally
        val = np.clip(data.qpos[j1] - step, 0.0, 0.04)
        data.ctrl[a1] = val
        data.ctrl[a2] = val
        print(f"✅ Gripper closing: {float(val):.4f}")

    except Exception as e:
        print(f"❌ Failed to close gripper: {e}")







def screw_fingers(step=0.002):
    try:
        a1 = model.actuator("finger1_act").id
        a2 = model.actuator("finger2_act").id
        data.ctrl[a1] += step
        data.ctrl[a2] -= step
        mujoco.mj_step(model, data)
        print("🔁 Fingers screw.")
    except Exception as e:
        print(f"❌ Failed to screw fingers: {e}")
def unscrew_fingers(step=0.002):
    try:
        a1 = model.actuator("finger1_act").id
        a2 = model.actuator("finger2_act").id
        data.ctrl[a1] -= step
        data.ctrl[a2] += step
        mujoco.mj_step(model, data)
        print("🔁 Fingers unscrew.")
    except Exception as e:
        print(f"❌ Failed to unscrew fingers: {e}")

# === Viewer loop ===
# === Viewer loop ===
#last_valid_gesture = -1
#gesture_hold_time = 0.5  # seconds to hold last gesture
#gesture_timeout = time.time()

last_action_time = time.time()
action_cooldown = 0.4  # seconds between allowed actions
# UDP setup to receive gestures
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(("127.0.0.1", 5005))  # <- Listen on 5006 (or any unused port)
udp_sock.setblocking(False)  # Non-blocking receive
menu_locked = False
current_mode = None  # e.g., 'x', 'y', 'z', 'gripper', 'finger', 'screw'
# 🔧 Initialize state tracking before viewer loop
#gesture_stable_start = None
#last_lock_candidate = None
#lock_hold_duration = 3  # seconds
menu_locked = False
current_mode = None
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🟢 Viewer launched. Use gestures to control the robot.")

    current_mode = None
    menu_locked = False
    last_action_time = 0

    stable_gesture = None
    stable_count = 0
    required_stable_count = 0

    while viewer.is_running():
        now = time.time()

        try:
            data_in, _ = udp_sock.recvfrom(1024)
            gesture = int(struct.unpack('d', data_in[12:20])[0])

            print(f"\n📨 Gesture received: {gesture}")
            print(f"   🔒 Locked: {menu_locked}, Mode: {current_mode}, Stable: {stable_gesture}, Count: {stable_count}")

            # ───────── Auto-lock ─────────
            if 20 <= gesture <= 25:
                if gesture == stable_gesture:
                    stable_count += 1
                    print(f"   🕒 Holding gesture {gesture} — stable count: {stable_count}")
                else:
                    if stable_gesture is not None:
                        print(f"   ⚠️ Gesture changed from {stable_gesture} to {gesture} before locking.")
                    stable_gesture = gesture
                    stable_count = 1

                if stable_count >= required_stable_count and not menu_locked:
                    mode_map = {
                        20: 'x', 21: 'y', 22: 'z',
                        23: 'gripper', 24: 'finger', 25: 'screw'
                    }
                    current_mode = mode_map[gesture]
                    menu_locked = True
                    print(f"   🔐 Axis locked to mode: {current_mode}")

            else:
                stable_gesture = None
                stable_count = 0

            # ───────── Manual unlock ─────────
            if gesture == 3:
                print("   🔓 Unlocking...")
                current_mode = None
                menu_locked = False
                stable_gesture = None
                stable_count = 0

            # ───────── Movement ─────────
            elif menu_locked and (now - last_action_time >= 0.3):
                print(f"   ✅ Executing movement for gesture {gesture} in mode {current_mode}")

                if gesture == 4:
                    if current_mode == 'x':
                        move_cartesian_custom(dx=STEP_SIZE)
                    elif current_mode == 'y':
                        move_cartesian_custom(dy=STEP_SIZE)
                    elif current_mode == 'z':
                        move_cartesian_custom(dz=STEP_SIZE)
                    elif current_mode == 'gripper':
                        rotate_gripper(+0.05)
                    elif current_mode == 'finger':
                        open_gripper()
                    elif current_mode == 'screw':
                        screw_fingers()

                elif gesture == 5:
                    if current_mode == 'x':
                        move_cartesian_custom(dx=-STEP_SIZE)
                    elif current_mode == 'y':
                        move_cartesian_custom(dy=-STEP_SIZE)
                    elif current_mode == 'z':
                        move_cartesian_custom(dz=-STEP_SIZE)
                    elif current_mode == 'gripper':
                        rotate_gripper(-0.05)
                    elif current_mode == 'finger':
                        close_gripper()
                    elif current_mode == 'screw':
                        unscrew_fingers()

                last_action_time = now

        except BlockingIOError:
            pass

        viewer.sync()
        time.sleep(0.01)











