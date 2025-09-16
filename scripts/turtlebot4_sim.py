# turtlebot4_sim.py
# Requires: pip install mujoco
# (Optional for the windowed viewer) a working OpenGL context.

import time
from dataclasses import dataclass
from typing import Callable, Dict, Any, Optional

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
                 sensor_hz: int = 50):
        assert sim_hz % sensor_hz == 0, "sim_hz must be a multiple of sensor_hz (e.g., 500 and 50)."
        self.sim_hz = sim_hz
        self.sensor_hz = sensor_hz
        self.steps_per_sensor = sim_hz // sensor_hz

        # Load model
        if from_string:
            self.model = mujoco.MjModel.from_xml_string(xml_path_or_str)
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path_or_str)

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

        # Build actuator name -> index map (handy if you want to send controls by name)
        self._act_id: Dict[str, int] = {}
        for i in range(self.model.nu):
            self._act_id[self.model.actuator(i).name] = i

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
    def read_all_sensors(self) -> Dict[str, Any]:
        """
        Return a dict: sensor_name -> scalar (float) or np.ndarray for vector sensors.
        """
        out: Dict[str, Any] = {}
        for s in self._sensors.values():
            vals = self.data.sensordata[s.adr: s.adr + s.dim]
            out[s.name] = float(vals[0]) if s.dim == 1 else np.array(vals, copy=True)
        return out

    # ---------- Main loop ----------
    def run(self,
            seconds: float = 10.0,
            realtime: bool = True,
            on_sample: Optional[Callable[[float, Dict[str, Any]], None]] = None) -> None:
        """
        Run the simulation.
          - seconds: how long to run
          - realtime: if True, tries to match wall-clock time
          - on_sample: callback called at 50 Hz: fn(sim_time, sensor_dict)
        """
        steps_total = int(round(seconds * self.sim_hz))
        next_wall_time = time.perf_counter()
        step_dt = 1.0 / self.sim_hz

        # Launch viewer if requested
        if self._use_viewer:
            # passive viewer: we control stepping
            with viewer.launch_passive(self.model, self.data) as v:
                self._viewer = v
                self._loop(steps_total, step_dt, realtime, on_sample)
                self._viewer = None
        else:
            self._loop(steps_total, step_dt, realtime, on_sample)

    def _loop(self, steps_total: int, step_dt: float, realtime: bool,
              on_sample: Optional[Callable[[float, Dict[str, Any]], None]]) -> None:

        # For pacing
        start = time.perf_counter()
        next_step_wall = start

        for step in range(steps_total):
            # Step physics once (1 / sim_hz seconds of sim time)
            mujoco.mj_step(self.model, self.data)

            # Keep viewer responsive if present
            if self._viewer is not None:
                # Clear any previous custom geoms and sync
                self._viewer.sync()

            # Sample sensors every N steps (to get 50 Hz)
            if (step + 1) % self.steps_per_sensor == 0:
                t = self.data.time
                sensors = self.read_all_sensors()
                if on_sample:
                    on_sample(t, sensors)
                else:
                    # Default: brief print (keep it lightweight)
                    # Show a couple of common sensors if they exist, otherwise count
                    demo_keys = [k for k in ("imu_ang", "imu_acc", "left_vel", "right_vel") if k in sensors]
                    if demo_keys:
                        brief = {k: sensors[k] for k in demo_keys}
                        print(f"[{t:7.3f}s] sample: {brief}")
                    else:
                        print(f"[{t:7.3f}s] sampled {len(sensors)} sensors.")

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

    sim = Turtlebot4Sim(XML_PATH, from_string=False, open_viewer=True, sim_hz=500, sensor_hz=50)

    # Optional: set some controls by actuator name (if present in your model)
    # sim.set_control(forward=0.0, turn=0.0)

    print("Sensors discovered:", sim.sensor_names())
    print("Actuators discovered:", sim.actuator_names())

    # Collect 5 seconds of data at 50 Hz while sim runs at 500 Hz
    def print_sample(t, sensors):
        # Minimal, human-readable output. Customize as needed.
        # left_v = sensors.get("left_vel", None)
        # right_v = sensors.get("right_vel", None)
        for key in ["scan00", "scan01", "scan02", "scan03", "scan04", "scan05", "scan06", "scan07", "scan08", "scan09","scan10", "scan11", "scan12", "scan13", "scan14", "scan15", "scan16", "scan17"]:
            s = sensors.get(key, None)
            print(f"{s}", end=", ")
        print("")
    sim.run(seconds=5.0, realtime=True, on_sample=print_sample)
