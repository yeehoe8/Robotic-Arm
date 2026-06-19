from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
import numpy as np
from scipy.optimize import minimize
from scipy.signal import butter, filtfilt
import serial

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def serve_dashboard():
    return FileResponse("index.html")

BOUNDS = [
    (np.radians(-90), np.radians(90)),     
    (np.radians(-60), np.radians(60)),     
    (np.radians(-55), np.radians(19)),     
    (np.radians(-135), np.radians(135)),   
    (np.radians(-90), np.radians(90)),     
    (np.radians(-90), np.radians(90))      
]

usb_serial = None
serial_buffer = ""

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

@app.get("/send_command")
def send_command(cmd: str):
    global usb_serial
    if usb_serial and usb_serial.is_open:
        usb_serial.write(f"<{cmd}>".encode('utf-8'))
        return {"status": "success"}
    return {"status": "failed"}

@app.get("/check_status")
def check_status():
    global usb_serial, serial_buffer
    if usb_serial and usb_serial.is_open:
        try:
            if usb_serial.in_waiting > 0:
                raw_data = usb_serial.read(usb_serial.in_waiting).decode('utf-8', errors='ignore')
                serial_buffer += raw_data
            if "OBJ_DETECTED" in serial_buffer:
                serial_buffer = "" 
                return {"status": "success", "message": "OBJ_DETECTED"}
            if len(serial_buffer) > 1000:
                serial_buffer = serial_buffer[-100:]
        except Exception:
            pass
    return {"status": "empty"}

@app.get("/set_hardware")
def set_hardware(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float, sol: int = 0):
    global usb_serial
    if usb_serial and usb_serial.is_open:
        pwms = [str(max(80, min(600, get_calibrated_pwm(i, q)))) for i, q in enumerate([q1, q2, q3, q4, q5, q6])]
        pwms.append(str(sol))
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

def jacobian_ik(target_xyz, start_thetas, max_iter=50, tol=1e-4):
    thetas = np.copy(start_thetas)
    damping = 0.05 
    for step in range(max_iter):
        T_cur, _ = build_dh_chain(thetas)
        pos_cur = T_cur[:3, 3]
        error_pos = target_xyz - pos_cur
        error_pitch = np.radians(-90.0) - (thetas[2] + thetas[4])
        error = np.array([error_pos[0], error_pos[1], error_pos[2], error_pitch, -thetas[3], -thetas[5]])
        
        if np.linalg.norm(error_pos) < tol: break
            
        J = np.zeros((6, 6))
        epsilon = 1e-5
        for i in range(6):
            thetas_eps = np.copy(thetas)
            thetas_eps[i] += epsilon
            J[:3, i] = (build_dh_chain(thetas_eps)[0][:3, 3] - pos_cur) / epsilon
        
        J[3, 2] = 1.0; J[3, 4] = 1.0; J[4, 3] = 1.0; J[5, 5] = 1.0                 
        J_pseudo = np.linalg.inv(J.T @ J + (damping**2) * np.eye(6)) @ J.T
        thetas += J_pseudo @ error
        
        for i in range(6): thetas[i] = np.clip(thetas[i], BOUNDS[i][0], BOUNDS[i][1])
    return thetas

def constraint_vertical(thetas):
    return (thetas[2] + thetas[4]) - np.radians(-90.0)

def find_best_ik(target_xyz, current_angles):
    # FIX: Smart trigonometry guess prevents the IK Solver from getting stuck in Local Minima
    q1_guess = np.arctan2(target_xyz[1], target_xyz[0])
    
    guesses = [
        np.radians(current_angles), 
        [q1_guess, np.radians(45), np.radians(-20), 0, np.radians(-70), 0], 
        [q1_guess, np.radians(60), np.radians(-40), 0, np.radians(-50), 0]
    ]
    best_res, best_err = None, float('inf')
    con = {'type': 'eq', 'fun': constraint_vertical}
    
    def objective_strict(thetas, target_xyz):
        T, _ = build_dh_chain(thetas)
        return np.sum((T[:3, 3] - target_xyz)**2) + (thetas[3]**2 + thetas[5]**2) * 50000.0

    def objective_relaxed(thetas, target_xyz):
        T, _ = build_dh_chain(thetas)
        pos_error = np.sum((T[:3, 3] - target_xyz)**2) * 1000.0
        pitch_error = abs((thetas[2] + thetas[4]) - np.radians(-90.0)) * 100.0
        return pos_error + pitch_error + (thetas[3]**2 + thetas[5]**2) * 10.0
        
    for guess in guesses:
        res = minimize(objective_strict, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, constraints=[con], options={'ftol': 1e-4, 'maxiter': 50})
        err = np.linalg.norm(build_dh_chain(res.x)[0][:3, 3] - target_xyz)
        if err < best_err:
            best_err, best_res = err, res
            
    if best_err > 10.0:
        best_err_relaxed, best_res_relaxed = float('inf'), None
        for guess in guesses:
            res = minimize(objective_relaxed, guess, args=(target_xyz,), method='SLSQP', bounds=BOUNDS, options={'ftol': 1e-4, 'maxiter': 50})
            err = np.linalg.norm(build_dh_chain(res.x)[0][:3, 3] - target_xyz)
            if err < best_err_relaxed:
                best_err_relaxed, best_res_relaxed = err, res
        if best_err_relaxed < best_err:
            best_err, best_res = best_err_relaxed, best_res_relaxed
            
    return best_res, best_err

# ==============================================================================
# SINGLE MASTER API ROUTER (Self-Contained)
# ==============================================================================
@app.get("/inverse_kinematics")
def calculate_ik(
    cur1: float, cur2: float, cur3: float, cur4: float, cur5: float, cur6: float,
    mode: str = "macro_auto",
    x: float = 0.0, y: float = 0.0, z: float = 0.0,
    px: float = 0.0, py: float = 0.0, pz: float = 0.0,
    dx: float = 0.0, dy: float = 0.0, dz: float = 0.0
):
    # --------------------------------------------------------------------------
    # INTERNAL SEQUENCE GENERATOR HELPERS
    # --------------------------------------------------------------------------
    def calc_delay_frames(start_angles, ms, label, sol):
        num_frames = max(1, int(ms / 33.33))
        frames = []
        _, points = build_dh_chain(np.radians(start_angles))
        frame_dict = {"angles": [round(float(a), 2) for a in start_angles], "joints": points, "label": label, "sol": sol}
        for _ in range(num_frames): frames.append(frame_dict)
        return frames, start_angles

    def calc_home_frames(start_angles, label, sol):
        target = [0.0] * 6
        max_delta = np.max(np.abs(np.array(target) - np.array(start_angles)))
        T = max(1.5, max_delta / 35.0)  
        tau = np.linspace(0, 1.0, int(T * 30))
        s_tau = 10*(tau**3) - 15*(tau**4) + 6*(tau**5)
        
        frames = []
        for s in s_tau:
            angs = [start_angles[j] + (target[j] - start_angles[j]) * s for j in range(6)]
            _, points = build_dh_chain(np.radians(angs))
            frames.append({"angles": [round(float(a), 2) for a in angs], "joints": points, "label": label, "sol": sol})
            
        return frames, target

    def calc_ptp_frames(start_angles, target_xyz, label, sol):
        best_res, best_err = find_best_ik(target_xyz, start_angles)
        if best_res is None or best_err > 10.0: return None, start_angles
        
        target_angles_deg = np.degrees(best_res.x)
        max_delta = np.max(np.abs(target_angles_deg - np.array(start_angles)))
        T = max(1.5, max_delta / 35.0)  
        tau = np.linspace(0, 1.0, int(T * 30))
        s_tau = 10*(tau**3) - 15*(tau**4) + 6*(tau**5)
        
        frames = []
        for s in s_tau:
            angs = start_angles + (target_angles_deg - start_angles) * s
            _, points = build_dh_chain(np.radians(angs))
            frames.append({"angles": [round(float(a), 2) for a in angs], "joints": points, "label": label, "sol": sol})
        return frames, target_angles_deg.tolist()

    def calc_lin_frames(start_angles, target_xyz, label, sol):
        start_xyz = build_dh_chain(np.radians(start_angles))[0][:3, 3]
        dist = np.linalg.norm(np.array(target_xyz) - start_xyz)
        if dist < 1.0: return calc_delay_frames(start_angles, 33, label, sol)

        best_res, best_err = find_best_ik(target_xyz, start_angles)
        if best_err > 10.0: return None, start_angles

        T = max(1.0, dist / 35.0)  
        tau = np.linspace(0, 1.0, int(T * 30))
        s_tau = 10*(tau**3) - 15*(tau**4) + 6*(tau**5)
        
        cartesian_path = [start_xyz + (np.array(target_xyz) - start_xyz) * s for s in s_tau]
        raw_angles, prev_guess = [], np.radians(start_angles)
        
        for pt in cartesian_path:
            thetas = jacobian_ik(pt, prev_guess)
            raw_angles.append(np.degrees(thetas))
            prev_guess = thetas

        raw_angles = np.array(raw_angles)
        raw_angles[-1] = np.degrees(best_res.x) 
        
        b, a = butter(N=2, Wn=0.1, btype='lowpass')
        smoothed_angles = np.zeros_like(raw_angles)
        if len(raw_angles) > 9: 
            for i in range(6): smoothed_angles[:, i] = filtfilt(b, a, raw_angles[:, i])
            smoothed_angles[-1] = np.degrees(best_res.x)
        else:
            smoothed_angles = raw_angles

        frames = []
        for angs in smoothed_angles:
            _, points = build_dh_chain(np.radians(angs))
            frames.append({"angles": [round(float(a), 2) for a in angs], "joints": points, "label": label, "sol": sol})
        return frames, smoothed_angles[-1].tolist()

    # --------------------------------------------------------------------------
    # API ROUTER LOGIC
    # --------------------------------------------------------------------------
    current = [cur1, cur2, cur3, cur4, cur5, cur6]
    frames = []

    T_start, _ = build_dh_chain(np.radians(current))
    current_xyz = T_start[:3, 3]

    # ROUTE 1: HOME MACRO
    if mode == "macro_home":
        # FIX: Only extract if we are down near the table.
        if current_xyz[2] < 300.0:
            extract_z = current_xyz[2] + 100.0 
            f, current = calc_lin_frames(current, [current_xyz[0], current_xyz[1], extract_z], "LIN (EXTRACT)", 0)
            if f: frames.extend(f)

        f, current = calc_home_frames(current, "PTP (HOMING)", 0)
        frames.extend(f)
        
        return {"status": "success", "frames": frames}

    # ROUTE 2: MANUAL POINT-TO-POINT + PLUNGE MACRO
    elif mode == "macro_manual":
        clearance_z = z + 100.0

        # FIX: Only extract if we are down near the table. Prevent drawing impossible vertical lines from Home.
        if current_xyz[2] < 300.0:
            extract_z = max(current_xyz[2] + 100.0, clearance_z)
            f, current = calc_lin_frames(current, [current_xyz[0], current_xyz[1], extract_z], "LIN (EXTRACT)", 0)
            if f: frames.extend(f)

        f, current = calc_ptp_frames(current, [x, y, clearance_z], "PTP (TRANSFER)", 0)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_lin_frames(current, [x, y, z], "LIN (PLUNGE)", 0)
        if not f: return {"status": "failed"}
        frames.extend(f)

        return {"status": "success", "frames": frames}

    # ROUTE 3: FULL AUTO PICK-AND-PLACE MACRO
    elif mode == "macro_auto":
        clearance_z = max(pz, dz) + 100.0

        if current_xyz[2] < 300.0:
            extract_z = max(current_xyz[2] + 100.0, clearance_z)
            f, current = calc_lin_frames(current, [current_xyz[0], current_xyz[1], extract_z], "LIN (EXTRACT)", 0)
            if f: frames.extend(f)

        f, current = calc_ptp_frames(current, [px, py, clearance_z], "PTP (TRANSFER)", 0)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_lin_frames(current, [px, py, pz], "LIN (PLUNGE)", 0)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_delay_frames(current, 600, "LIN (GRAB)", 1)
        frames.extend(f)

        f, current = calc_lin_frames(current, [px, py, clearance_z], "LIN (EXTRACT)", 1)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_ptp_frames(current, [dx, dy, clearance_z], "PTP (TRANSFER)", 1)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_lin_frames(current, [dx, dy, dz], "LIN (PLACE)", 1)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_delay_frames(current, 500, "LIN (RELEASE)", 0)
        frames.extend(f)

        f, current = calc_lin_frames(current, [dx, dy, clearance_z], "LIN (EXTRACT)", 0)
        if not f: return {"status": "failed"}
        frames.extend(f)

        f, current = calc_home_frames(current, "PTP (HOMING)", 0)
        frames.extend(f)

        return {"status": "success", "frames": frames, "trigger_resume": True}

    return {"status": "failed", "message": "Invalid Mode"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
