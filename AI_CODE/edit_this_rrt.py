import numpy as np
from collections import deque
try:
    from project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory
except ImportError:
    # PyTest import.
    from .project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory

# custom UTILS is just a file where you can put your own custom functions to import into the controller
try:
    import dejan_custom_utils as dcu  # dcu stands for dejan custom utils
except ImportError:
    # PyTest import.
    from . import dejan_custom_utils as dcu


class Controller():
    #Template controller class.

    # initializes the controller
    def __init__(self,
                 initial_obs,
                 initial_info,
                 use_firmware: bool = False,
                 buffer_size: int = 100,
                 verbose: bool = False
                 ):
        # Save environment and control parameters.
        self.CTRL_TIMESTEP = initial_info["ctrl_timestep"]
        self.CTRL_FREQ = initial_info["ctrl_freq"]
        self.initial_obs = initial_obs
        self.VERBOSE = verbose
        self.BUFFER_SIZE = buffer_size

        # Store a priori scenario information.
        self.NOMINAL_GATES = initial_info["nominal_gates_pos_and_type"]
        self.NOMINAL_OBSTACLES = initial_info["nominal_obstacles_pos"]

        # Check for pycffirmware.
        if use_firmware:
            self.ctrl = None  # do not use PID controller, instead use the built in firmware controller
        else:
            self.ctrl = PIDController()
            self.KF = initial_info["quadrotor_kf"]

        # Reset counters and buffers.
        self.reset()
        self.interEpisodeReset()

        # perform trajectory planning
        t_scaled = self.planning(use_firmware, initial_info)

        # visualization
        plot_trajectory(t_scaled, self.waypoints, self.ref_x, self.ref_y, self.ref_z)
        draw_trajectory(initial_info, self.waypoints, self.ref_x, self.ref_y, self.ref_z)

    def planning(self, use_firmware, initial_info):
        """Trajectory planning algorithm"""
        start_xyz = np.array([
            self.initial_obs[0],
            self.initial_obs[2],
            initial_info["gate_dimensions"]["tall"]["height"] if use_firmware else self.initial_obs[4],
        ], dtype=float)
        final_xyz = np.array([
            initial_info["x_reference"][0],
            initial_info["x_reference"][2],
            initial_info["x_reference"][4],
        ], dtype=float)

        bounds = {
            "x": (-3.2, 3.2),
            "y": (-3.2, 3.2),
        }

        raw_waypoints = dcu.plan_full_mission(
            start_xyz=start_xyz,
            gates=self.NOMINAL_GATES,
            obstacles=self.NOMINAL_OBSTACLES,
            final_xyz=final_xyz,
            bounds=bounds,
            inflated_radius=0.33,
            step_size=0.28,
            search_radius=0.65,
            max_iter=2500,
            rng_seed=7,
        )

        self.waypoints = raw_waypoints
        dense_path = dcu.resample_polyline_with_z(raw_waypoints, spacing=0.06)
        self.ref_x = dense_path[:, 0]
        self.ref_y = dense_path[:, 1]
        self.ref_z = np.clip(dense_path[:, 2], 0.15, 1.95)

        path_len = np.sum(np.linalg.norm(np.diff(dense_path, axis=0), axis=1)) if len(dense_path) > 1 else 0.0
        cruise_speed = 0.7
        duration = max(8.0, path_len / cruise_speed)
        t_scaled = np.linspace(0.0, duration, len(self.ref_x))
        self.TRAJ_DURATION = duration

        if self.VERBOSE:
            print(f"Planned {len(self.waypoints)} sparse waypoints, {len(self.ref_x)} reference points, duration {duration:.2f}s")

        return t_scaled

    def cmdFirmware(self,
                    time,
                    obs,
                    reward=None,
                    done=None,
                    info=None
                    ):
        if self.ctrl is not None:
            raise RuntimeError("[ERROR] Using method 'cmdFirmware' but Controller was created with 'use_firmware' = False.")

        iteration = int(time*self.CTRL_FREQ)
        traj_start_iter = int(3*self.CTRL_FREQ)
        traj_end_iter = traj_start_iter + len(self.ref_x)

        if iteration == 0:
            height = float(max(0.6, self.ref_z[0]))
            duration = 2
            command_type = Command.TAKEOFF
            args = [height, duration]

        elif traj_start_iter <= iteration < traj_end_iter:
            step = min(iteration - traj_start_iter, len(self.ref_x) - 1)
            target_pos = np.array([self.ref_x[step], self.ref_y[step], self.ref_z[step]])

            if step < len(self.ref_x) - 1:
                vel = (np.array([self.ref_x[step + 1], self.ref_y[step + 1], self.ref_z[step + 1]]) - target_pos) * self.CTRL_FREQ
            else:
                vel = np.zeros(3)

            if step < len(self.ref_x) - 2:
                vel_next = (np.array([self.ref_x[step + 2], self.ref_y[step + 2], self.ref_z[step + 2]]) - np.array([self.ref_x[step + 1], self.ref_y[step + 1], self.ref_z[step + 1]])) * self.CTRL_FREQ
                acc = (vel_next - vel) * self.CTRL_FREQ
            else:
                acc = np.zeros(3)

            target_yaw = float(np.arctan2(vel[1], vel[0])) if np.linalg.norm(vel[:2]) > 1e-6 else 0.0
            target_rpy_rates = np.zeros(3)

            command_type = Command.FULLSTATE
            args = [target_pos, vel, acc, target_yaw, target_rpy_rates]

        elif iteration == traj_end_iter:
            command_type = Command.NOTIFYSETPOINTSTOP
            args = []

        elif iteration == traj_end_iter + 1:
            x = float(self.ref_x[-1])
            y = float(self.ref_y[-1])
            z = float(self.ref_z[-1])
            yaw = 0.0
            duration = 1.5
            command_type = Command.GOTO
            args = [[x, y, z], yaw, duration, False]

        elif iteration == traj_end_iter + int(2*self.CTRL_FREQ):
            height = 0.0
            duration = 3.0
            command_type = Command.LAND
            args = [height, duration]

        elif iteration == traj_end_iter + int(5*self.CTRL_FREQ):
            command_type = Command.STOP
            args = []

        else:
            command_type = Command.NONE
            args = []

        return command_type, args

    def cmdSimOnly(self,
                   time,
                   obs,
                   reward=None,
                   done=None,
                   info=None
                   ):
        if self.ctrl is None:
            raise RuntimeError("[ERROR] Attempting to use method 'cmdSimOnly' but Controller was created with 'use_firmware' = True.")

        iteration = int(time*self.CTRL_FREQ)

        if iteration < len(self.ref_x):
            target_p = np.array([self.ref_x[iteration], self.ref_y[iteration], self.ref_z[iteration]])
        else:
            target_p = np.array([self.ref_x[-1], self.ref_y[-1], self.ref_z[-1]])
        target_v = np.zeros(3)

        return target_p, target_v

    def reset(self):
        self.action_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.obs_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.reward_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.done_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.info_buffer = deque([], maxlen=self.BUFFER_SIZE)

        self.interstep_counter = 0
        self.interepisode_counter = 0

    def interEpisodeReset(self):
        self.interstep_learning_time = 0
        self.interstep_learning_occurrences = 0
        self.interepisode_learning_time = 0
