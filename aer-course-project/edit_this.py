import numpy as np
from collections import deque
try:
    from project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory
except ImportError:
    # PyTest import.
    from .project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory

# custom UTILS is just a file where you can put your own cutsom functions to import into the controller
try:
    import dejan_custom_utils as dcu # dcu stands for dejan custom utils
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
        """Initialization of the controller.

        INSTRUCTIONS:
            The controller's constructor has access the initial state `initial_obs` and the a priori infromation
            contained in dictionary `initial_info`. Use this method to initialize constants, counters, pre-plan
            trajectories, etc.

        Args:
            initial_obs (ndarray): The initial observation of the quadrotor's state
                [x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r].
            initial_info (dict): The a priori information as a dictionary with keys
                'symbolic_model', 'nominal_physical_parameters', 'nominal_gates_pos_and_type', etc.
            use_firmware (bool, optional): Choice between the on-board controll in `pycffirmware`
                or simplified software-only alternative.
            buffer_size (int, optional): Size of the data buffers used in method `learn()`.
            verbose (bool, optional): Turn on and off additional printouts and plots.

        """
        # Save environment and control parameters.
        self.CTRL_TIMESTEP = initial_info["ctrl_timestep"]
        self.CTRL_FREQ = initial_info["ctrl_freq"]
        self.initial_obs = initial_obs
        self.VERBOSE = verbose
        self.BUFFER_SIZE = buffer_size

        # Store a priori scenario information.
        # plan the trajectory based on the information of the (1) gates and (2) obstacles. 
        self.NOMINAL_GATES = initial_info["nominal_gates_pos_and_type"]
        self.NOMINAL_OBSTACLES = initial_info["nominal_obstacles_pos"]

        # Check for pycffirmware.
        if use_firmware:
            self.ctrl = None # do not use PID controller, instead use the built in firmware controller
        else:
            # Initialize a simple PID Controller for debugging and test.
            # Do NOT use for the IROS 2022 competition. 
            self.ctrl = PIDController()
            # Save additonal environment parameters.
            self.KF = initial_info["quadrotor_kf"]

        # Reset counters and buffers.
        self.reset()
        self.interEpisodeReset()

        # perform trajectory planning
        t_scaled = self.planning(use_firmware, initial_info)

        ## visualization
        # Plot trajectory in each dimension and 3D.
        plot_trajectory(t_scaled, self.waypoints, self.ref_x, self.ref_y, self.ref_z)

        # Draw the trajectory on PyBullet's GUI.
        draw_trajectory(initial_info, self.waypoints, self.ref_x, self.ref_y, self.ref_z)

    # this function is where we will put our trajectory, important part is that you define the ref_x, ref_y, and ref_z variables
    def planning(self, use_firmware, initial_info):
        """Trajectory planning algorithm"""
 
        # Example Code Provided By Course
        '''
        # Call a function in module `example_custom_utils`.
        #ecu.exampleFunction() # currently function is notn used and does nothing so I commented it out

        # initial waypoint (initial waypoint is like the starting position, kinda weird but 0 is for x, 2 is for y, and 4 appears to be for z. Not sure where this initial observation is coming form))
        if use_firmware: # honestly not too sure what "use_firmware" means but it ajusts the starting height of the drone, depensing on if it's on or off
            waypoints = [(self.initial_obs[0], self.initial_obs[2], initial_info["gate_dimensions"]["tall"]["height"])]  # Height is hardcoded scenario knowledge.
        else:
            waypoints = [(self.initial_obs[0], self.initial_obs[2], self.initial_obs[4])]

        # Example code: hardcode waypoints 
        waypoints.append((-0.5, -3.0, 2.0))
        waypoints.append((-0.5, -2.0, 2.0))
        waypoints.append((-0.5, -1.0, 2.0))
        waypoints.append((-0.5,  0.0, 2.0))
        waypoints.append((-0.5,  1.0, 2.0))
        waypoints.append((-0.5,  2.0, 2.0))
        waypoints.append([initial_info["x_reference"][0], initial_info["x_reference"][2], initial_info["x_reference"][4]]) # I have 0 clue where this x_refernce is comong from 

        # Polynomial fit.
        self.waypoints = np.array(waypoints)
        deg = 6 # 6th degree polynomial fit 
        t = np.arange(self.waypoints.shape[0]) # t DOESNT represent TIME, just a counter for the waypoints, but it gets converted to time like 5 lines down
        fx = np.poly1d(np.polyfit(t, self.waypoints[:,0], deg))
        fy = np.poly1d(np.polyfit(t, self.waypoints[:,1], deg))
        fz = np.poly1d(np.polyfit(t, self.waypoints[:,2], deg))
        duration = 15
        t_scaled = np.linspace(t[0], t[-1], int(duration*self.CTRL_FREQ)) % t converted to actual time
        self.ref_x = fx(t_scaled)
        self.ref_y = fy(t_scaled)
        self.ref_z = fz(t_scaled)
        '''

        '''
        # Lab 2 Code
        # Above is the provided example code, now I will try to make the circle
        # Attempt #3: Using Waypoints
        circle_radius = 1 # m
        circle_center = (0, -3, 1) # m
        # t = np.arange(30)
        
        # adding sampling points info
        duration = 15 # s (try making this faster after I'm curious what will happen)
        t_scaled = np.linspace(0, duration, int(duration*self.CTRL_FREQ))
        angle_values = np.linspace(0, 2*np.pi, int(duration*self.CTRL_FREQ))
        
        # defining the functions of the path
        self.ref_x = np.cos(angle_values)*circle_radius + circle_center[0]
        self.ref_y = np.sin(angle_values)*circle_radius + circle_center[1]
        self.ref_z = np.ones(len(angle_values)) * circle_center[2]

        self.waypoints = np.array([
            [circle_center[0], circle_center[1], circle_center[2]],
            ])
        '''
        
        # Final Project Code
        #dcu.exampleFunction() # currently function is not used and does nothing so I commented it out
        gates_order = [1, 2, 3]
        duration = 19 # seconds, becuase 20 seconds is hard coded into cmdFirmware(), you must change it before you can make this greater than 20 seconds
        t_scaled = np.linspace(0, duration, int(duration*self.CTRL_FREQ)) # covering the entire time duration
        time_per_gate = duration / len(gates_order)
        step_per_gate = floor(time_per_gate * self.CTRL_FREQ) # have to floor it because you can't have a fraction of a step

        for i in gates_order:
            gate_pos = ...
            starting_step = i * step_per_gate

            for step in range(starting_step, starting_step + step_per_gate):








        return t_scaled

    # this function is only used when we use the high level controls from the crazyswarm library which do everything for you basically, if this runs cmdSimOnly will not run
    def cmdFirmware(self,
                    time,
                    obs,
                    reward=None,
                    done=None,
                    info=None
                    ):
        """Pick command sent to the quadrotor through a Crazyswarm/Crazyradio-like interface.

        INSTRUCTIONS:
            Re-implement this method to return the target position, velocity, acceleration, attitude, and attitude rates to be sent
            from Crazyswarm to the Crazyflie using, e.g., a `cmdFullState` call.

        Args:
            time (float): Episode's elapsed time, in seconds.
            obs (ndarray): The quadrotor's Vicon data [x, 0, y, 0, z, 0, phi, theta, psi, 0, 0, 0].
            reward (float, optional): The reward signal.
            done (bool, optional): Wether the episode has terminated.
            info (dict, optional): Current step information as a dictionary with keys
                'constraint_violation', 'current_target_gate_pos', etc.

        Returns:
            Command: selected type of command (takeOff, cmdFullState, etc., see Enum-like class `Command`).
            List: arguments for the type of command (see comments in class `Command`)

        """
        # This line below makes it so if you are using the PID Controller, you will get an error, this function only works if you are using the built in high level crazy fly commmands
        if self.ctrl is not None: 
            raise RuntimeError("[ERROR] Using method 'cmdFirmware' but Controller was created with 'use_firmware' = False.")

        # [INSTRUCTIONS] 
        # self.CTRL_FREQ is 30 (set in the getting_started.yaml file) 
        # control input iteration indicates the number of control inputs sent to the quadrotor
        iteration = int(time*self.CTRL_FREQ) # converts time to control steps, ie. at iteartion 90, the time is 3 seconds

        #########################
        # REPLACE THIS (START) ##
        #########################
        
        # print("The info. of the gates ")
        # print(self.NOMINAL_GATES)

        if iteration == 0:
            height = 1
            duration = 2

            command_type = Command(2)  # Take-off. # Command 2 is the takeoff command (crazyswarm build in function)
            args = [height, duration] # this function is called at every time step, so at the end of this function it returns command type and args

        # [INSTRUCTIONS] Example code for using cmdFullState interface   
        # Runs between 3 and 20 seconds
        elif iteration >= 3*self.CTRL_FREQ and iteration < 20*self.CTRL_FREQ:
            step = min(iteration-3*self.CTRL_FREQ, len(self.ref_x) -1) # the min is just for protection, this line just tells you what step you are on
            target_pos = np.array([self.ref_x[step], self.ref_y[step], self.ref_z[step]])
            target_vel = np.zeros(3) # you are not providing any desired velocity or acceleration commands, only position
            target_acc = np.zeros(3) 
            target_yaw = 0. # you are not providing any desired yaw or angular rate commands, only position
            target_rpy_rates = np.zeros(3)

            command_type = Command(1)  # cmdFullState.
            args = [target_pos, target_vel, target_acc, target_yaw, target_rpy_rates]

        # Runs at 20 seconds, just to run command 6 which lets you go back from low level control to high level control
        elif iteration == 20*self.CTRL_FREQ:
            command_type = Command(6)  # Notify setpoint stop.
            args = []

       # [INSTRUCTIONS] Example code for using goTo interface 
       # at 20 seconds + 1 step, you use the high level control to go the last point on the path, in this case end of ref_x and ref_y
        elif iteration == 20*self.CTRL_FREQ+1:
            x = self.ref_x[-1]
            y = self.ref_y[-1]
            z = 1.5 
            yaw = 0.
            duration = 2.5

            command_type = Command(5)  # goTo.
            args = [[x, y, z], yaw, duration, False]

        # at 23 seconds runs another goTo command to go back to the starting position (takes 6 seconds)
        elif iteration == 23*self.CTRL_FREQ:
            x = self.initial_obs[0]
            y = self.initial_obs[2]
            z = 1.5
            yaw = 0.
            duration = 6

            command_type = Command(5)  # goTo.
            args = [[x, y, z], yaw, duration, False]

        # at 30 seconds, you land the drone with command 3
        elif iteration == 30*self.CTRL_FREQ:
            height = 0.
            duration = 3

            command_type = Command(3)  # Land.
            args = [height, duration]

        # at 33 seconds - 1 step, you send the stop command to the drone to stop the trajectory
        elif iteration == 33*self.CTRL_FREQ-1:
            command_type = Command(4)  # STOP command to be sent once the trajectory is completed.
            args = []

        # For all other cases, you do nothing
        else:
            command_type = Command(0)  # None.
            args = []

        #########################
        # REPLACE THIS (END) ####
        #########################

        return command_type, args

    # this function runs ONLY WHEN YOU ARE NOT USING cmdFirmware(), honeselty no clue when we would ever use this but this basically just lets you tru it with the build in PID controller 
    def cmdSimOnly(self,
                   time,
                   obs,
                   reward=None,
                   done=None,
                   info=None
                   ):
        """PID per-propeller thrusts with a simplified, software-only PID quadrotor controller.

        INSTRUCTIONS:
            You do NOT need to re-implement this method for the project.
            Only re-implement this method when `use_firmware` == False to return the target position and velocity.

        Args:
            time (float): Episode's elapsed time, in seconds.
            obs (ndarray): The quadrotor's state [x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r].
            reward (float, optional): The reward signal.
            done (bool, optional): Wether the episode has terminated.
            info (dict, optional): Current step information as a dictionary with keys
                'constraint_violation', 'current_target_gate_pos', etc.

        Returns:
            List: target position (len == 3).
            List: target velocity (len == 3).

        """
        if self.ctrl is None:
            raise RuntimeError("[ERROR] Attempting to use method 'cmdSimOnly' but Controller was created with 'use_firmware' = True.")

        iteration = int(time*self.CTRL_FREQ)

        #########################
        if iteration < len(self.ref_x):
            target_p = np.array([self.ref_x[iteration], self.ref_y[iteration], self.ref_z[iteration]])
        else:
            target_p = np.array([self.ref_x[-1], self.ref_y[-1], self.ref_z[-1]])
        target_v = np.zeros(3)
        #########################

        return target_p, target_v

    # just resets the buffers and counters
    def reset(self):
        """Initialize/reset data buffers and counters.

        Called once in __init__().

        """
        # Data buffers.
        self.action_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.obs_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.reward_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.done_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.info_buffer = deque([], maxlen=self.BUFFER_SIZE)

        # Counters.
        self.interstep_counter = 0
        self.interepisode_counter = 0

    # NOTE: this function is not used in the course project. 
    def interEpisodeReset(self):
        """Initialize/reset learning timing variables.

        Called between episodes in `getting_started.py`.

        """
        # Timing stats variables.
        self.interstep_learning_time = 0
        self.interstep_learning_occurrences = 0
        self.interepisode_learning_time = 0
