import can
import time
from tinymovr.bus_router import init_router
from tinymovr.config import get_bus_config, create_device

params = get_bus_config(["canine", "slcan_disco"], bitrate=1000000)
init_router(can.Bus, params)

tm1 = create_device(node_id=1)
tm2 = create_device(node_id=2)

# Calibrate motors 
def initialize_motor(motor):
    print(f"Calibrating motor {motor.comms.can.id}...")
    motor.controller.calibrate()
    time.sleep(25)  # Wait for calibration to complete
    print(f"Calibration for motor {motor.comms.can.id} done!")
    motor.controller.current.Iq_setpoint = 0.0
    motor.controller.current_mode()
    motor.controller.current.Iq_limit = 2.0  # Limit max current for safety (max current of motor)

initialize_motor(tm1)
initialize_motor(tm2)

# Set the current physical position as virtual zero by saving it to sensors.user_frame.offset
tm1.sensors.user_frame.offset = tm1.sensors.user_frame.position_estimate
tm2.sensors.user_frame.offset = tm2.sensors.user_frame.position_estimate

# Gains for the bilateral teleoperation control loop
Kp = 30           # Position gain
Kd = 0.2           # Coupling damping gain
K_local = 0.1     # Local damping

# Torque constant (Nm/A)
Kt = 1.4043083213990768 

TICKS_TO_RAD = (2.0 * 3.141592653589793) / 65536.0

def get_angle_and_velocity(motor):
    pos_ticks = motor.sensors.user_frame.position_estimate.magnitude
    vel_ticks_s = motor.sensors.user_frame.velocity_estimate.magnitude
    pos = pos_ticks * TICKS_TO_RAD
    vel = vel_ticks_s * TICKS_TO_RAD
    return pos, vel

dt = 0.001  # Target 1 kHz update rate, actual 300..400Hz

loop_counter = 0
last_print_time = time.time()

try:
    while True:
        t0 = time.time()
        
        # Read positions and velocities
        theta1, dtheta1 = get_angle_and_velocity(tm1)
        theta2, dtheta2 = get_angle_and_velocity(tm2)
        
        # Compute torques based on coupling equations
        tau1 = Kp * (theta2 - theta1) + Kd * (dtheta2 - dtheta1) - K_local * dtheta1
        tau2 = Kp * (theta1 - theta2) + Kd * (dtheta1 - dtheta2) - K_local * dtheta2
        
        # Send computed torques as current setpoints
        tm1.controller.current.Iq_setpoint = tau1
        tm2.controller.current.Iq_setpoint = tau2
        
        # Maintain the loop 
        elapsed = time.time() - t0
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        loop_counter += 1
        current_time = time.time()
        if current_time - last_print_time >= 1.0:
            # Convert currents from A to mA
            iq_setpoint_tm1 = round(tm1.controller.current.Iq_setpoint.magnitude * 1000)
            iq_estimate_tm1 = round(tm1.controller.current.Iq_estimate.magnitude * 1000)
            iq_setpoint_tm2 = round(tm2.controller.current.Iq_setpoint.magnitude * 1000)
            iq_estimate_tm2 = round(tm2.controller.current.Iq_estimate.magnitude * 1000)
            
            # Convert currents to torque using Kt
            torque_setpoint_tm1 = iq_setpoint_tm1 * (Kt/1000)
            torque_estimate_tm1 = iq_estimate_tm1 * (Kt/1000)
            torque_setpoint_tm2 = iq_setpoint_tm2 * (Kt/1000)
            torque_estimate_tm2 = iq_estimate_tm2 * (Kt/1000)
            
            print(
                f"Loop frequency: {loop_counter} Hz | "
                f"Motor {tm1.comms.can.id} - Iq setpoint: {round(iq_setpoint_tm1 * 1000)} mA, "
                f"Iq estimate: {round(iq_estimate_tm1 * 1000)} mA, "
                f"Torque setpoint: {torque_setpoint_tm1:.4f} Nm, "
                f"Torque estimate: {torque_estimate_tm1:.4f} Nm | "
                f"Motor {tm2.comms.can.id} - Iq setpoint: {round(iq_setpoint_tm2 * 1000)} mA, "
                f"Iq estimate: {round(iq_estimate_tm2 * 1000)} mA, "
                f"Torque setpoint: {torque_setpoint_tm2:.4f} Nm, "
                f"Torque estimate: {torque_estimate_tm2:.4f} Nm"
            )
            loop_counter = 0
            last_print_time = current_time
except KeyboardInterrupt:
    print("Exiting safely...")
    tm1.idle()
    tm2.idle()
