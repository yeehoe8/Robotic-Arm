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

# Explicitly defining the true physical limits
BOUNDS = [
    (np.radians(-90), np.radians(65.5)),   
    (np.radians(-60), np.radians(60)),     
    (np.radians(-55), np.radians(19)),     
    (np.radians(-135), np.radians(135)),   
    (np.radians(-90), np.radians(90)),     
    (np.radians(-90), np.radians(90))      
]

# TILT COMPENSATION: Fine-tune this if the arm still sags slightly under its own weight
J5_TILT_COMPENSATION_DEG = 0.0  

usb_serial = None

def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# ==============================================================================
# PIECEWISE HARDWARE CALIBRATION MAP
# This perfectly translates the Virtual 3D Angle to the Real-World PWM Signal
# anchored exactly to the Home (0 degree) position.
# ==============================================================================
def get_calibrated_pwm(joint_idx, angle):
    # JOINT 1: Base Yaw (Home = 324)
    if joint_idx == 0:
        if angle >= 0: return int(map_val(angle, 0.0, 45.0, 324.0, 411.5))
        else:          return int(map_val(angle, 0.0, -90.0, 324.0, 131.5))
        
    # JOINT 2: Shoulder Pitch (Home = 319)
    elif joint_idx == 1:
        if angle >= 0: return int(map_val(angle, 0.0, 45.0, 319.0, 381.5))
        else:          return int(map_val(angle, 0.0, -45.0, 319.0, 256.5))
        
    # JOINT 3: Elbow Pitch (Home = 131, Reverse Direction)
    elif joint_idx == 2:
        if angle >= 0: return int(map_val(angle, 0.0, 20.0, 131.0, 106.0))
        else:          return int(map_val(angle, 0.0, -45.0, 131.0, 193.5))
        
    # JOINT 4: Wrist Yaw (Home = 287)
    elif joint_idx == 3:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 287.0, 412.0))
        else:          return int(map_val(angle, 0.0, -90.0, 287.0, 162.0))
        
    # JOINT 5: Wrist Pitch (Home = 299)
    elif joint_idx == 4:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 299.0, 524.0))
        else:          return int(map_val(angle, 0.0, -90.0, 299.0, 118.0))
        
    # JOINT 6: Suction Roll (Home = 307)
    elif joint_idx == 5:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 307.0, 532.0))
        # Corrected overshoot: -90 degrees UI previously gave -100 physical
        else:          return int(map_val(angle, 0.0, -90.0, 307.0, 104.5))
        
    return 300 # Failsafe

@app.get("/connect_serial")
def connect_serial(port: str):
    global usb_serial
    try:
        if usb_serial and usb_serial.is_open:
            usb_serial.close()
        usb_serial = serial.Serial(port, 115200, timeout=1)
        return {"status": "success"}
    except Exception as e:
        return {"status": "failed", "message": str(e)}

@app.get("/set_hardware")
def set_hardware(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float):
    global usb_serial
    if usb_serial and usb_serial.is_open:
        virtual_angles = [q1, q2, q3, q4, q5, q6]
        pwms = []
        for i in range(6):
            # Pass the virtual angle directly into our new exact calibration map
            pwm_val = get_calibrated_pwm(i, virtual_angles[i])
            
            # Absolute hardware clamps to prevent stripped gears
            pwm_val = max(80, min(600, pwm_val)) 
            pwms.append(str(pwm_val))
            
        command = "<" + ",".join(pwms) + ">"
        usb_serial.write(command.encode('utf-8'))
        return {"status": "success"}
    return {"status": "failed"}

def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def build_dh_chain(thetas):
    math_thetas = np.copy(thetas)
    math_thetas[1] += (np.pi / 2) 
    math_thetas[2] -= thetas[1] # Parallel Linkage Fix
    
    dh_params = [
        [30,  np.pi/2,  125, math_thetas[0]], [160, 0, 0, math_thetas[1]], [50,  np.pi/2, 0, math_thetas[2]],
        [0, -np.pi/2, 285, math_thetas[3]], [0, np.pi/2, 0, math_thetas[4]], [0, 0, 80, math_thetas[5]] 
    ]
    
    T_current = np.eye(4)
    points = [[0.0, 0.0, 0.0]]
    for param in dh_params:
        T_current = T_current @ dh_matrix(*param)
        points.append([float(T_current[0,3]), float(T_current[1,3]), float(T_current[2,3])])
        
    return T_current, points

# --- STAGE 1: STRICT POSITIONAL SOLVER ---
def objective_strict(thetas, target_xyz):
    T, _ = build_dh_chain(thetas)
    pos_error = np.sum((T[:3, 3] - target_xyz)**2)
    
    # Penalize J4 and J6 to prevent the wrist from twisting sideways
    twist_penalty = (thetas[3]**2 + thetas[5]**2) * 50000.0
    return pos_error + twist_penalty

# --- THE UNBREAKABLE ALGEBRAIC CONSTRAINT (WITH DROOP COMPENSATION) ---
def constraint_vertical(thetas):
    target_pitch = np.radians(-90.0 + J5_TILT_COMPENSATION_DEG)
    return (thetas[2] + thetas[4]) - target_pitch

# --- STAGE 2: RELAXED FALLBACK SOLVER ---
def objective_relaxed(thetas, target_xyz):
    T, _ = build_dh_chain(thetas)
    pos_error = np.sum((T[:3, 3] - target_xyz)**2) * 1000.0
    
    target_pitch = np.radians(-90.0 + J5_TILT_COMPENSATION_DEG)
    pitch_error = abs((thetas[2] + thetas[4]) - target_pitch) * 100.0
    twist_penalty = (thetas[3]**2 + thetas[5]**2) * 10.0
    return pos_error + pitch_error + twist_penalty

@app.get("/kinematics")
def get_kinematics(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float):
    _, points = build_dh_chain(np.radians([q1, q2, q3, q4, q5, q6]))
    return {"status": "success", "joints": points}

# --- STANDARD POINT-TO-POINT INVERSE KINEMATICS ---
@app.get("/inverse_kinematics")
def calculate_ik(x: float, y: float, z: float, cur1: float, cur2: float, cur3: float, cur4: float, cur5: float, cur6: float):
    target_xyz = np.array([x, y, z])
    current_angles = [cur1, cur2, cur3, cur4, cur5, cur6]
    
    guesses = [
        np.radians(current_angles),
        np.radians([0, 45, -20, 0, -70, 0]),  
        np.radians([0, 60, -40, 0, -50, 0]),  
        np.radians([0, 30, -55, 0, -35, 0])   
    ]

    best_res_strict = None
    best_err_strict = float('inf')
    
    # The dictionary required by SLSQP to enforce our exact vertical math
    con = {'type': 'eq', 'fun': constraint_vertical}
    
    for guess in guesses:
        res = minimize(objective_strict, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, constraints=[con], options={'ftol': 1e-4, 'maxiter': 100})
        T_res, _ = build_dh_chain(res.x)
        true_error = np.linalg.norm(T_res[:3, 3] - target_xyz)
        
        if true_error < best_err_strict:
            best_err_strict = true_error
            best_res_strict = res

    if best_err_strict <= 10.0:
        best_result = best_res_strict
        best_error = best_err_strict
    else:
        best_res_relaxed = None
        best_err_relaxed = float('inf')
        
        for guess in guesses:
            res = minimize(objective_relaxed, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, options={'ftol': 1e-4, 'maxiter': 100})
            T_res, _ = build_dh_chain(res.x)
            true_error = np.linalg.norm(T_res[:3, 3] - target_xyz)
            
            if true_error < best_err_relaxed:
                best_err_relaxed = true_error
                best_res_relaxed = res
                
        best_result = best_res_relaxed
        best_error = best_err_relaxed

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
            _, frame_points = build_dh_chain(np.radians(frame_angles))
            animation_frames.append({
                "angles": [round(float(a), 2) for a in frame_angles],
                "joints": frame_points
            })

        status_msg = "success" if best_error <= 5.0 else "warning"
        return {"status": status_msg, "frames": animation_frames, "error_mm": round(best_error, 1)}
    else:
        return {"status": "failed"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
