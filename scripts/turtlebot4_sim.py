# turtlebot4_sim.py
# Requires: pip install mujoco
# (Optional for the windowed viewer) a working OpenGL context.

import time
from dataclasses import dataclass
from typing import Callable, Dict, Any, Optional, Tuple

import numpy as np
import mujoco
try:
    from mujoco import viewer  # optional
    HAS_VIEWER = True
except Exception:
    HAS_VIEWER = False


@dataclass
class SensorInfo:
    name: str
    adr: int
    dim: int


class Turtlebot4Sim:
    """
    Simple, maintainable wrapper for your TurtleBot4 MJCF:
      - sim at 500 Hz
      - sensor reads at 50 Hz
      - optional viewer
    """

    def __init__(self,
                 xml_path_or_str: str,
                 from_string: bool = False,
                 open_viewer: bool = False,
                 sim_hz: int = 500,
                 control_hz: int = 50):
        assert sim_hz % control_hz == 0, "sim_hz must be a multiple of sensor_hz (e.g., 500 and 50)."
        self.sim_hz = sim_hz
        self.control_hz = control_hz
        self.steps_per_sensor = sim_hz // control_hz
        self.last_scan = []
        self.sensors: Dict[str, Any] = {}
        self.t = 0.0  # sim time

        # Load model
        if from_string:
            self.model = mujoco.MjModel.from_xml_string(xml_path_or_str)
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path_or_str)

        # Sensors/Actuators to be presnet:
        self.tb4_sensors = ['imu_quat', 'imu_ang', 'imu_acc', 'left_pos', 'left_vel', 'right_pos', 'right_vel', 'lidar_yaw_pos', 'lidar_yaw_vel', 'scan']
        self.tb4_actuators = ['forward', 'turn', 'lidar_spin']
        
        # Override timestep to hit desired sim rate (500 Hz => 0.002 s)
        self.model.opt.timestep = 1.0 / float(self.sim_hz)

        self.data = mujoco.MjData(self.model)

        # Build sensor index map (name -> slice in sensordata)
        self._sensors: Dict[str, SensorInfo] = {}
        for i in range(self.model.nsensor):
            name = self.model.sensor(i).name
            adr = int(self.model.sensor_adr[i])
            dim = int(self.model.sensor_dim[i])
            self._sensors[name] = SensorInfo(name=name, adr=adr, dim=dim)
        
        # Check that expected sensors are present
        for s in self.tb4_sensors:
            if s not in self._sensors:
                print(f"[warn] Expected sensor '{s}' not found in model.")
                exit(1)

        # Build actuator name -> index map (handy if you want to send controls by name)
        self._act_id: Dict[str, int] = {}
        for i in range(self.model.nu):
            self._act_id[self.model.actuator(i).name] = i
        
        # Check that expected actuators are present
        for a in self.tb4_actuators:
            if a not in self._act_id:
                print(f"[warn] Expected actuator '{a}' not found in model.")
                exit(1)
        
        # Initialise lidar rotation
        self.data.ctrl[self._act_id["lidar_spin"]] = float(0.05)  # ~2.5 Hz spin

        # Optional viewer
        self._viewer = None
        self._use_viewer = bool(open_viewer)
        if self._use_viewer and not HAS_VIEWER:
            print("[warn] mujoco.viewer not available; running headless.")
            self._use_viewer = False

    # ---------- Controls ----------
    def set_control(self, **kwargs: float) -> None:
        """
        Set actuator controls by name. Example:
          sim.set_control(forward=0.1, turn=-0.2)
        """
        for name, value in kwargs.items():
            if name not in self._act_id:
                raise KeyError(f"Actuator '{name}' not found. Available: {list(self._act_id.keys())}")
            self.data.ctrl[self._act_id[name]] = float(value)

    def zero_controls(self) -> None:
        if self.model.nu > 0:
            self.data.ctrl[:] = 0.0

    # ---------- Sensors ----------
    def read_sensors(self) -> Dict[str, Any]:
        """
        Return a dict: sensor_name -> scalar (float) or np.ndarray for vector sensors.
        """
        out: Dict[str, Any] = {}
        for s in self.tb4_sensors[0:7]:  # first 7 are vectors
            info = self._sensors[s]
            vals = self.data.sensordata[info.adr: info.adr + info.dim]
            out[s] = np.array(vals, copy=True)

        return out

    def read_scan(self) -> Tuple[float, float]:
        """
        Read lidar orientation and rangefinder value.
        """
        yaw_pos_info = self._sensors['lidar_yaw_pos']
        scan_info = self._sensors['scan']

        yaw_pos = float(self.data.sensordata[yaw_pos_info.adr]) % (2 * np.pi)
        scan = float(self.data.sensordata[scan_info.adr])

        return (yaw_pos, scan)

    # ---------- Main loop ----------    
    def iterate(self, realtime: bool) -> None:

        # For pacing
        next_step_wall = time.perf_counter()
        sim_steps = int(self.sim_hz / self.control_hz)
        step_dt = 1.0 / self.control_hz
        self.last_scan = []

        for _ in range(sim_steps): # steps per control
            # Step physics once (1 / sim_hz seconds of sim time)
            mujoco.mj_step(self.model, self.data)
            # Read lidar scan
            self.last_scan.append(self.read_scan())
    
        # Sample sensors at control_hz rate
        self.t = self.data.time
        self.sensors = self.read_sensors()
        
        # Keep viewer responsive if present
        if self._viewer is not None:
            # Clear any previous custom geoms and sync
            self._viewer.sync()

        # Optional real-time pacing (keeps ~sim_hz timing)
        if realtime:
            next_step_wall += step_dt
            now = time.perf_counter()
            sleep_s = next_step_wall - now
            if sleep_s > 0:
                time.sleep(sleep_s)

    # ---------- Utility ----------
    def sensor_names(self):
        return list(self._sensors.keys())

    def actuator_names(self):
        return list(self._act_id.keys())


# ---------------- Example usage ----------------
if __name__ == "__main__":
    # Change this to your file path, or set from_string=True and paste the XML string.
    XML_PATH = "scene.xml"

    sim = Turtlebot4Sim(XML_PATH, from_string=False, open_viewer=True, sim_hz=500, control_hz=10)

    # Optional: set some controls by actuator name (if present in your model)
    # sim.set_control(forward=0.0, turn=0.0)

    print("Sensors discovered:", sim.sensor_names())
    print("Actuators discovered:", sim.actuator_names())

    # Collect 5 seconds of data at 50 Hz while sim runs at 500 Hz
     # Launch viewer if requested
    if sim._use_viewer:
        # passive viewer: we control stepping
        with viewer.launch_passive(sim.model, sim.data) as v:
            sim._viewer = v
            for _ in range(50):  # run for 5 seconds at 10 Hz
                sim.iterate(realtime=True)
                #publish IMU message, and joint states message with the info in sim.sensors and the stam self.t
                print(f"t={sim.t:.2f} imu_quat={sim.sensors['imu_quat']} left_vel={sim.sensors['left_vel']} right_vel={sim.sensors['right_vel']} last_scan={sim.last_scan[-1]}") 
                # publish lidar scan message with the info in sim.last_scan
                print(f"last_scan={sim.last_scan}")
                # ros spin once and if a new control message is received in topic /cmd_vel, call sim.set_control(forward=..., turn=...)
                sim.set_control(forward=0.3, turn=1.0)  # example constant forward command
            sim._viewer = None
    else:
        for _ in range(50):  # run for 5 seconds at 10 Hz
            sim.iterate(realtime=False)