from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import numpy as np
from scipy.optimize import minimize
import serial
from scipy.signal import savgol_filter
from scipy.signal import butter, filtfilt

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Explicitly defining the true physical limits
BOUNDS = [
    (np.radians(-90), np.radians(90)),     
    (np.radians(-60), np.radians(60)),     
    (np.radians(-55), np.radians(19)),     
    (np.radians(-135), np.radians(135)),   
    (np.radians(-90), np.radians(90)),     
    (np.radians(-90), np.radians(90))      
]

usb_serial = None

def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def get_calibrated_pwm(joint_idx, angle):
    if joint_idx == 0:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 324.0, 499.0)) 
        else:          return int(map_val(angle, 0.0, -90.0, 324.0, 131.5))
    elif joint_idx == 1:
        if angle >= 0: return int(map_val(angle, 0.0, 45.0, 319.0, 381.5))
        else:          return int(map_val(angle, 0.0, -45.0, 319.0, 256.5))
    elif joint_idx == 2:
        if angle >= 0: return int(map_val(angle, 0.0, 20.0, 131.0, 106.0))
        else:          return int(map_val(angle, 0.0, -45.0, 131.0, 193.5))
    elif joint_idx == 3:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 287.0, 412.0))
        else:          return int(map_val(angle, 0.0, -90.0, 287.0, 162.0))
    elif joint_idx == 4:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 299.0, 524.0))
        else:          return int(map_val(angle, 0.0, -90.0, 299.0, 118.0))
    elif joint_idx == 5:
        if angle >= 0: return int(map_val(angle, 0.0, 90.0, 307.0, 532.0))
        else:          return int(map_val(angle, 0.0, -90.0, 307.0, 104.5))
    return 300 

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
        pwms = [str(max(80, min(600, get_calibrated_pwm(i, q)))) for i, q in enumerate([q1, q2, q3, q4, q5, q6])]
        usb_serial.write(("<" + ",".join(pwms) + ">").encode('utf-8'))
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
    math_thetas[2] -= thetas[1] 
    
    dh_params = [
        [30,  np.pi/2,  125, math_thetas[0]], [160, 0, 0, math_thetas[1]], [50,  np.pi/2, 0, math_thetas[2]],
        [0, -np.pi/2, 285, math_thetas[3]], [0, np.pi/2, 0, math_thetas[4]], [0, 0, 75, math_thetas[5]] 
    ]
    
    T_current = np.eye(4)
    points = [[0.0, 0.0, 0.0]]
    for param in dh_params:
        T_current = T_current @ dh_matrix(*param)
        points.append([float(T_current[0,3]), float(T_current[1,3]), float(T_current[2,3])])
        
    return T_current, points

@app.get("/kinematics")
def get_kinematics(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float):
    _, points = build_dh_chain(np.radians([q1, q2, q3, q4, q5, q6]))
    return {"status": "success", "joints": points}
# ------------------------------------------

# ==============================================================================
# DIFFERENTIAL KINEMATICS: DAMPED LEAST SQUARES JACOBIAN ENGINE
# ==============================================================================
def jacobian_ik(target_xyz, start_thetas, max_iter=50, tol=1e-4):
    """
    Uses the Differential Jacobian Matrix to solve micro-movements flawlessly.
    Enforces a strict -90 degree downward pitch constraint for suction cups.
    """
    thetas = np.copy(start_thetas)
    damping = 0.05 # Levenberg-Marquardt damping factor to prevent singularity crashes
    
    for step in range(max_iter):
        T_cur, _ = build_dh_chain(thetas)
        pos_cur = T_cur[:3, 3]
        
        # 1. Calculate Cartesian Error
        error_pos = target_xyz - pos_cur
        
        # 2. Calculate Orientation Error (Force downward pitch, prevent wrist twist)
        error_pitch = np.radians(-90.0) - (thetas[2] + thetas[4])
        error_t3 = 0.0 - thetas[3]
        error_t5 = 0.0 - thetas[5]
        
        # 6-Degree Error Vector
        error = np.array([error_pos[0], error_pos[1], error_pos[2], error_pitch, error_t3, error_t5])
        
        # If we have reached the target within tolerance, stop calculating
        if np.linalg.norm(error_pos) < tol:
            break
            
        # 3. Build the 6x6 Jacobian Matrix using Finite Differences
        J = np.zeros((6, 6))
        epsilon = 1e-5
        
        for i in range(6):
            thetas_eps = np.copy(thetas)
            thetas_eps[i] += epsilon
            T_eps, _ = build_dh_chain(thetas_eps)
            # Spatial XYZ derivatives
            J[:3, i] = (T_eps[:3, 3] - pos_cur) / epsilon
        
        # Analytical derivatives for orientation constraints
        J[3, 2] = 1.0; J[3, 4] = 1.0  # Pitch constraint derivative
        J[4, 3] = 1.0                 # Roll constraint derivative
        J[5, 5] = 1.0                 # Yaw constraint derivative
        
        # 4. Damped Least Squares Inverse (J_pseudo = (J^T * J + lambda^2 * I)^-1 * J^T)
        J_T = J.T
        J_pseudo = np.linalg.inv(J_T @ J + (damping**2) * np.eye(6)) @ J_T
        
        # 5. Calculate joint deltas and update
        delta_theta = J_pseudo @ error
        thetas += delta_theta
        
        # 6. Enforce physical hardware bounds
        for i in range(6):
            thetas[i] = np.clip(thetas[i], BOUNDS[i][0], BOUNDS[i][1])
            
    return thetas

# ==============================================================================
# GLOBAL MACRO-SOLVER (For large sweeps across the desk)
# ==============================================================================
def constraint_vertical(thetas):
    return (thetas[2] + thetas[4]) - np.radians(-90.0)

def objective_strict(thetas, target_xyz):
    T, _ = build_dh_chain(thetas)
    return np.sum((T[:3, 3] - target_xyz)**2) + (thetas[3]**2 + thetas[5]**2) * 50000.0

def find_best_ik(target_xyz, current_angles):
    guesses = [
        np.radians(current_angles),
        np.radians([0, 45, -20, 0, -70, 0]),  
        np.radians([0, 60, -40, 0, -50, 0])  
    ]
    best_res_strict = None
    best_err_strict = float('inf')
    con = {'type': 'eq', 'fun': constraint_vertical}
    
    for guess in guesses:
        res = minimize(objective_strict, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, constraints=[con], options={'ftol': 1e-4, 'maxiter': 50})
        T_res, _ = build_dh_chain(res.x)
        true_error = np.linalg.norm(T_res[:3, 3] - target_xyz)
        if true_error < best_err_strict:
            best_err_strict = true_error
            best_res_strict = res
            
    return best_res_strict, best_err_strict

# ==============================================================================
# DYNAMIC TRAJECTORY PLANNER
# ==============================================================================
@app.get("/inverse_kinematics")
def calculate_ik(x: float, y: float, z: float, cur1: float, cur2: float, cur3: float, cur4: float, cur5: float, cur6: float, mode: str = "auto_plunge"):
    target_xyz = np.array([x, y, z])
    current_angles = [cur1, cur2, cur3, cur4, cur5, cur6]
    animation_frames = []

    if mode == "auto_plunge":
        T_start, _ = build_dh_chain(np.radians(current_angles))
        current_xyz = T_start[:3, 3]

        lift_xyz = np.copy(current_xyz); lift_xyz[2] += 40.0 
        hover_xyz = np.copy(target_xyz); hover_xyz[2] += 40.0 

        # Broad global solutions for macro-targets
        best_res_lift, err_lift = find_best_ik(lift_xyz, current_angles)
        best_res_hover, err_hover = find_best_ik(hover_xyz, best_res_lift.x if best_res_lift is not None else current_angles)
        best_res_final, err_final = find_best_ik(target_xyz, best_res_hover.x if best_res_hover is not None else current_angles)

        if best_res_hover is None or err_hover > 25.0: return {"status": "failed", "message": "Hover coordinate out of reach."}
        if best_res_final is None or err_final > 25.0: return {"status": "failed", "message": "Final drop coordinate out of reach."}

        do_lift = best_res_lift is not None and err_lift <= 25.0

        # ==========================================
        # PHASE 1: CARTESIAN LIFT (JACOBIAN SOLVER)
        # ==========================================
        if do_lift:
            T_lift = 0.8 
            waypoints_lift = int(T_lift * 30)
            tau_lift = np.linspace(0, 1.0, waypoints_lift)
            s_tau_lift = 10*(tau_lift**3) - 15*(tau_lift**4) + 6*(tau_lift**5)

            cartesian_lift_path = [current_xyz + (lift_xyz - current_xyz) * s for s in s_tau_lift]
            
            raw_lift_angles = []
            prev_guess = np.radians(current_angles) 
            
            # Using the fast Jacobian Matrix for frame-by-frame solving
            for pt in cartesian_lift_path:
                thetas = jacobian_ik(pt, prev_guess)
                raw_lift_angles.append(np.degrees(thetas))
                prev_guess = thetas

            raw_lift_angles = np.array(raw_lift_angles)
            raw_lift_angles[-1] = np.degrees(best_res_lift.x) # Boundary Lock

            b, a = butter(N=2, Wn=0.1, btype='lowpass')
            smoothed_lift = np.zeros_like(raw_lift_angles)
            for i in range(6): smoothed_lift[:, i] = filtfilt(b, a, raw_lift_angles[:, i])
            smoothed_lift[-1] = np.degrees(best_res_lift.x)

            for frame_angles in smoothed_lift:
                _, frame_points = build_dh_chain(np.radians(frame_angles))
                animation_frames.append({"angles": [round(float(a), 2) for a in frame_angles], "joints": frame_points})
            
            start_sweep_angles = np.degrees(best_res_lift.x)
        else:
            start_sweep_angles = current_angles

        # ==========================================
        # PHASE 2: JOINT-SPACE SWEEP 
        # ==========================================
        hover_angles_deg = np.degrees(best_res_hover.x)
        max_delta_1 = np.max(np.abs(hover_angles_deg - np.array(start_sweep_angles)))
        T1 = max(1.5, max_delta_1 / 35.0)  
        waypoints1 = int(T1 * 30)
        tau1 = np.linspace(0, 1.0, waypoints1)
        s_tau1 = 10*(tau1**3) - 15*(tau1**4) + 6*(tau1**5)
        
        joint_profiles_1 = []
        for q_start, q_end in zip(start_sweep_angles, hover_angles_deg):
            joint_profiles_1.append(q_start + (q_end - q_start) * s_tau1)
        joint_profiles_1 = np.array(joint_profiles_1).T  
        
        for frame_angles in joint_profiles_1:
            _, frame_points = build_dh_chain(np.radians(frame_angles))
            animation_frames.append({"angles": [round(float(a), 2) for a in frame_angles], "joints": frame_points})

        # ==========================================
        # PHASE 3: CARTESIAN DROP (JACOBIAN SOLVER)
        # ==========================================
        T2 = 0.4 
        waypoints2 = int(T2 * 30)
        tau2 = np.linspace(0, 1.0, waypoints2)
        s_tau2 = 10*(tau2**3) - 15*(tau2**4) + 6*(tau2**5)
        
        cartesian_drop_path = [hover_xyz + (target_xyz - hover_xyz) * s for s in s_tau2]
        
        raw_drop_angles = []
        prev_guess = best_res_hover.x 
        
        # Using the fast Jacobian Matrix for frame-by-frame solving
        for pt in cartesian_drop_path:
            thetas = jacobian_ik(pt, prev_guess)
            raw_drop_angles.append(np.degrees(thetas))
            prev_guess = thetas

        raw_drop_angles = np.array(raw_drop_angles)
        raw_drop_angles[-1] = np.degrees(best_res_final.x) # Boundary Lock

        b, a = butter(N=2, Wn=0.1, btype='lowpass')
        smoothed_drop = np.zeros_like(raw_drop_angles)
        for i in range(6): smoothed_drop[:, i] = filtfilt(b, a, raw_drop_angles[:, i])
        smoothed_drop[-1] = np.degrees(best_res_final.x)

        for frame_angles in smoothed_drop:
            _, frame_points = build_dh_chain(np.radians(frame_angles))
            animation_frames.append({"angles": [round(float(a), 2) for a in frame_angles], "joints": frame_points})

        final_T, _ = build_dh_chain(np.radians(smoothed_drop[-1]))
        err_final = np.linalg.norm(final_T[:3, 3] - target_xyz)

        status_msg = "success" if err_final <= 5.0 else "warning"
        return {"status": status_msg, "frames": animation_frames, "error_mm": round(err_final, 1)}
    
    elif mode == "safe_home":
        # Get exact current Cartesian position
        T_start, _ = build_dh_chain(np.radians(current_angles))
        current_xyz = T_start[:3, 3]
        
        # Define the 40mm vertical lift target
        lift_xyz = np.copy(current_xyz)
        lift_xyz[2] += 40.0 
        
        # Verify the lift coordinate doesn't violate workspace bounds
        best_res_lift, err_lift = find_best_ik(lift_xyz, current_angles)
        do_lift = best_res_lift is not None and err_lift <= 25.0
        
        home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # ==========================================
        # PHASE 1: CARTESIAN LIFT (JACOBIAN)
        # ==========================================
        if do_lift:
            T_lift = 0.8  # 0.8 seconds duration
            waypoints_lift = int(T_lift * 30)
            tau_lift = np.linspace(0, 1.0, waypoints_lift)
            s_tau_lift = 10*(tau_lift**3) - 15*(tau_lift**4) + 6*(tau_lift**5)

            # Generate straight-line path in task space
            cartesian_lift_path = [current_xyz + (lift_xyz - current_xyz) * s for s in s_tau_lift]
            
            raw_lift_angles = []
            prev_guess = np.radians(current_angles) 
            
            for pt in cartesian_lift_path:
                thetas = jacobian_ik(pt, prev_guess)
                raw_lift_angles.append(np.degrees(thetas))
                prev_guess = thetas

            raw_lift_angles = np.array(raw_lift_angles)
            raw_lift_angles[-1] = np.degrees(best_res_lift.x) # Boundary Lock

            # Apply lowpass filter to smooth micro-jitters
            b, a = butter(N=2, Wn=0.1, btype='lowpass')
            smoothed_lift = np.zeros_like(raw_lift_angles)
            for i in range(6): smoothed_lift[:, i] = filtfilt(b, a, raw_lift_angles[:, i])
            smoothed_lift[-1] = np.degrees(best_res_lift.x)

            for frame_angles in smoothed_lift:
                _, frame_points = build_dh_chain(np.radians(frame_angles))
                animation_frames.append({"angles": [round(float(a), 2) for a in frame_angles], "joints": frame_points})
            
            # Start homing from the top of the lift
            start_home_angles = smoothed_lift[-1]
        else:
            # Fallback if the lift mathematically fails (e.g., already at max Z reach)
            start_home_angles = current_angles

        # ==========================================
        # PHASE 2: JOINT-SPACE SWEEP TO HOME
        # ==========================================
        max_delta = np.max(np.abs(np.array(home_angles) - np.array(start_home_angles)))
        T_home = max(1.5, max_delta / 35.0)  
        waypoints_home = int(T_home * 30)
        
        tau_home = np.linspace(0, 1.0, waypoints_home)
        s_tau_home = 10*(tau_home**3) - 15*(tau_home**4) + 6*(tau_home**5)
        
        joint_profiles = []
        for q_start, q_end in zip(start_home_angles, home_angles):
            joint_profiles.append(q_start + (q_end - q_start) * s_tau_home)
        joint_profiles = np.array(joint_profiles).T  
        
        for frame_angles in joint_profiles:
            _, frame_points = build_dh_chain(np.radians(frame_angles))
            animation_frames.append({"angles": [round(float(a), 2) for a in frame_angles], "joints": frame_points})
            
        return {"status": "success", "frames": animation_frames, "error_mm": 0.0}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
