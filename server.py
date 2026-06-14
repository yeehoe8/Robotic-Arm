from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import numpy as np
from scipy.optimize import minimize
import serial

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- MASTERING & HARDWARE CONFIGURATION ---
HOME_OFFSETS = [69.6, 67.6, -7.6, 54.8, 59.6, 62.8]
DIRECTIONS = [1, 1, -1, 1, 1, 1]

# Explicitly defining the physical limits
BOUNDS = [
    (np.radians(-90), np.radians(65.5)),   # Joint 1
    (np.radians(-60), np.radians(60)),     # Joint 2
    (np.radians(-55), np.radians(19)),     # Joint 3 (Restored to 19 degrees)
    (np.radians(-135), np.radians(135)),   # Joint 4
    (np.radians(-90), np.radians(90)),     # Joint 5
    (np.radians(-90), np.radians(90))      # Joint 6
]

bt_serial = None

@app.get("/connect_bt")
def connect_bt(port: str):
    global bt_serial
    try:
        if bt_serial and bt_serial.is_open:
            bt_serial.close()
        bt_serial = serial.Serial(port, 9600, timeout=1)
        return {"status": "success"}
    except Exception as e:
        return {"status": "failed", "message": str(e)}

@app.get("/set_hardware")
def set_hardware(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float):
    global bt_serial
    if bt_serial and bt_serial.is_open:
        virtual_angles = [q1, q2, q3, q4, q5, q6]
        pwms = []
        for i in range(6):
            physical_angle = (virtual_angles[i] * DIRECTIONS[i]) + HOME_OFFSETS[i]
            pwm_val = int(150 + (physical_angle * 2.5))
            
            # FIXED CLAMP: Lowered minimum bound to 80.
            # This allows Joint 3 to utilize positive angles (which drop below PWM 131).
            pwm_val = max(80, min(600, pwm_val)) 
            pwms.append(str(pwm_val))
            
        command = "<" + ",".join(pwms) + ">"
        bt_serial.write(command.encode('utf-8'))
        return {"status": "success"}
    return {"status": "failed"}

# --- CUSTOM MATHEMATICAL KINEMATICS ENGINE ---
def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def get_fk_full(thetas):
    math_thetas = np.copy(thetas)
    math_thetas[1] += (np.pi / 2) 
    
    dh_params = [
        [30,  np.pi/2,  125, math_thetas[0]],
        [160, 0,        0,   math_thetas[1]],
        [50,  np.pi/2,  0,   math_thetas[2]],
        [0,  -np.pi/2,  285, math_thetas[3]],
        [0,   np.pi/2,  0,   math_thetas[4]],
        [0,   0,        80,  math_thetas[5]] 
    ]
    
    T_current = np.eye(4)
    for param in dh_params:
        T_current = T_current @ dh_matrix(*param)
        
    return T_current

# STAGE 1: Strict Vertical Priority
def objective_strict(thetas, target_xyz):
    T = get_fk_full(thetas)
    current_xyz = T[:3, 3]
    tool_vector = T[:3, 2] 
    target_vector = np.array([0, 0, -1]) 
    
    pos_error = np.linalg.norm(current_xyz - target_xyz) * 10.0
    ori_error = np.linalg.norm(tool_vector - target_vector) * 5000.0
    twist_penalty = (abs(thetas[3]) + abs(thetas[5])) * 1000.0
    
    return pos_error + ori_error + twist_penalty

# STAGE 2: Relaxed Position Priority (Allows wrist to tilt)
def objective_relaxed(thetas, target_xyz):
    T = get_fk_full(thetas)
    current_xyz = T[:3, 3]
    tool_vector = T[:3, 2] 
    target_vector = np.array([0, 0, -1]) 
    
    # Position is absolute king
    pos_error = np.linalg.norm(current_xyz - target_xyz) * 10000.0
    
    # Mild suggestion to point down, but easily overriden by position
    ori_error = np.linalg.norm(tool_vector - target_vector) * 1.0
    # Keep wires from tangling
    twist_penalty = (abs(thetas[3]) + abs(thetas[5])) * 10.0
    
    return pos_error + ori_error + twist_penalty

# --- FORWARD KINEMATICS (Sliders) ---
@app.get("/kinematics")
def get_kinematics(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float):
    thetas = np.radians([q1, q2, q3, q4, q5, q6])
    thetas[1] += (np.pi / 2)
    
    dh_params = [
        [30,  np.pi/2,  125, thetas[0]], [160, 0, 0, thetas[1]], [50,  np.pi/2, 0, thetas[2]],
        [0, -np.pi/2, 285, thetas[3]], [0, np.pi/2, 0, thetas[4]], [0, 0, 80, thetas[5]] 
    ]
    
    points = [[0.0, 0.0, 0.0]]
    T_current = np.eye(4)
    for param in dh_params:
        T_current = T_current @ dh_matrix(*param)
        points.append([float(T_current[0,3]), float(T_current[1,3]), float(T_current[2,3])])
        
    return {"status": "success", "joints": points}

# --- TWO-STAGE CASCADED INVERSE KINEMATICS ---
@app.get("/inverse_kinematics")
def calculate_ik(x: float, y: float, z: float, cur1: float, cur2: float, cur3: float, cur4: float, cur5: float, cur6: float):
    target_xyz = np.array([x, y, z])
    current_angles = [cur1, cur2, cur3, cur4, cur5, cur6]
    
    guesses = [
        np.radians(current_angles),
        np.radians([0, 45, -20, 0, 65, 0]),  
        np.radians([0, 60, -40, 0, 70, 0]),  
        np.radians([0, 10,  10, 0, 70, 0])   
    ]

    # --- STAGE 1: Attempt Strict Vertical Solve ---
    best_res_strict = None
    best_err_strict = float('inf')
    
    for guess in guesses:
        res = minimize(objective_strict, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, options={'ftol': 1e-4, 'maxiter': 100})
        true_pos = get_fk_full(res.x)[:3, 3]
        true_error = np.linalg.norm(true_pos - target_xyz)
        
        if true_error < best_err_strict:
            best_err_strict = true_error
            best_res_strict = res

    # Evaluate Stage 1: Did we hit the coordinate within 10mm?
    if best_err_strict <= 10.0:
        best_result = best_res_strict
        best_error = best_err_strict
        print(f"IK Stage 1 Success: Reached target with VERTICAL LOCK. Missed by {best_error:.1f}mm")
    else:
        # --- STAGE 2: Fallback to Relaxed Position Solve ---
        print(f"IK Stage 1 Failed (Missed by {best_err_strict:.1f}mm). Falling back to Position-Only solve...")
        best_res_relaxed = None
        best_err_relaxed = float('inf')
        
        for guess in guesses:
            res = minimize(objective_relaxed, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, options={'ftol': 1e-4, 'maxiter': 100})
            true_pos = get_fk_full(res.x)[:3, 3]
            true_error = np.linalg.norm(true_pos - target_xyz)
            
            if true_error < best_err_relaxed:
                best_err_relaxed = true_error
                best_res_relaxed = res
                
        best_result = best_res_relaxed
        best_error = best_err_relaxed
        print(f"IK Stage 2 Complete: Reached target IGNORING ORIENTATION. Missed by {best_error:.1f}mm")

    # BEST EFFORT EXECUTION: Allow execution if within 25mm of target
    if best_result is not None and best_error < 25.0:
        target_angles_deg = np.degrees(best_result.x)
        
        T = 2.0  
        num_frames = 40 
        time_steps = np.linspace(0, T, num_frames)
        
        joint_profiles = []
        for q_start, q_end in zip(current_angles, target_angles_deg):
            a0 = q_start
            a3 = 10 * (q_end - q_start) / (T**3)
            a4 = -15 * (q_end - q_start) / (T**4)
            a5 = 6 * (q_end - q_start) / (T**5)
            
            profile = a0 + a3*(time_steps**3) + a4*(time_steps**4) + a5*(time_steps**5)
            joint_profiles.append(profile)
            
        joint_profiles = np.array(joint_profiles).T  
        
        animation_frames = []
        for frame_angles in joint_profiles:
            thetas_rad = np.radians(frame_angles)
            thetas_rad[1] += (np.pi / 2)
            
            dh_params = [
                [30,  np.pi/2,  125, thetas_rad[0]], [160, 0, 0, thetas_rad[1]], [50,  np.pi/2, 0, thetas_rad[2]],
                [0, -np.pi/2, 285, thetas_rad[3]], [0, np.pi/2, 0, thetas_rad[4]], [0, 0, 80, thetas_rad[5]]
            ]
            
            frame_points = [[0.0, 0.0, 0.0]]
            T_current = np.eye(4)
            for param in dh_params:
                T_current = T_current @ dh_matrix(*param)
                frame_points.append([float(T_current[0,3]), float(T_current[1,3]), float(T_current[2,3])])
                
            animation_frames.append({
                "angles": [round(float(a), 2) for a in frame_angles],
                "joints": frame_points
            })

        # Return "success" so the frontend JS executes the trajectory
        return {"status": "success", "frames": animation_frames, "error_mm": round(best_error, 1)}
    
    else:
        return {"status": "failed"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
