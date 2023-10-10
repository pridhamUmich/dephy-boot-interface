# Modified by M. Wu 2023, based on study_one.py by P. Pridham, 2020.

import time 
import numpy as np 
from ZhangCollins import *
from GaitCalculator import *

class FixedCollins:
    def __init__(self, user_mass, left_boot, right_boot):
        """Initialize fixed controller based on the Zhang/Collins profile. 

        Parameters
        ----------
        user_mass: user's mass in kg (?)
        left_boot: reference to left boot using Exo class from Exo.py 
        right_boot: reference to right boot using Exo class from Exo.py
        """
        self.user_mass = user_mass
        self.left_boot = left_boot
        self.right_boot = right_boot
        self.controller = ZhangCollins()
        self.left_gait = GaitCalculator()
        self.right_gait = GaitCalculator()
        self.start_time = time.monotonic()	# initialize the time for keeping track of when each trial started, this will be overwritten when the trial starts.
        self.feedback_mode = -1             # initialize training feedback mode to -1 (free exploration) -- 0 for metronome, 1 for strategy feedback 
       
        # parameters for Zhang/Collins profile 
        rspg = 0
        opg = 27.1
        ppg = 52.4
        spg = 62.7
        ot = 2
        npt = .175
		
        # initialize the Zhang/Collins profile
        self.controller.set_parameters(user_mass = self.user_mass, ramp_start_percent_gait = rspg, 
                                       onset_percent_gait = opg, peak_percent_gait = ppg, 
                                       stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)        
       
    def check_time(self, segment_min, restart_trial = False):
        """"Checks if the trial is currently in session and returns the index of the current trial. 

        Parameters
        ----------
        segment_min: list of duration of the each segment in minutes 
        restart_trial: boolean indicating if the trial should be restarted (default=False)

        Output
        ----------
        current_idx: the index of the current trial segment (-1 if trial has ended)
        """
        segment_sec = [x * 60 for x in segment_min]                         # convert from trial durations from minutes to seconds 
        time_points = np.cumsum(segment_sec)                                # get timepoints for the beginning of each trial segment

        if restart_trial:
            self.start_time = time.monotonic()                              # reset trial start time 

        current_time = time.monotonic()
        time_elapsed = current_time - self.start_time                       # calculate time elapsed since start of trial 
        time_passed = (time_elapsed) > time_points                          # logical list indicating which trial segment we are in 
        not_passed = [i for i, val in enumerate(time_passed) if not val]    # find indices of trial segments which have not passed 

        if not_passed:
            current_idx = not_passed[0]                                     # if the trial has not ended, return current trial segment index 
        else:
            current_idx = -1
        
        return current_idx
    
    def trial_handler(self, feedback_mode, use_torque):
        """Handles running of specific trial segments.

        Parameters
        ----------
        feedback_mode: label for feedback types used during training (0 for metronome, 1 for strategy feedback, -1 for free exploration or testing)
        use_torque: indicates if exokeleton should apply torque (1 for torque, 0 for slack/no torque)
        """

        self.feedback_mode = feedback_mode                                  # Need to provide audio feedback in external program 
        self.left_boot.read_data()                                          # Updates exoskeleton data
        self.left_gait.calculate_gait_parameters()                          # Approximates percent_gait, expected gait duration, etc
        self.right_boot.read_data()
        self.right_gait.calculate_gait_parameters()

        if use_torque == 1:
            # apply appropriate torques according to Zhang/Collins profile 

            left_tau = self.controller.calculate_torque_cmd(percent_gait = self.left_gait.percent_gait)
            right_tau = self.controller.calculate_torque_cmd(percent_gait = self.right_gait.percent_gait)
            self.left_boot.send_torque(left_tau)        # Applies torque according to gait percent and Zhang/Collins profile 
            self.right_boot.send_torque(right_tau)
        else:
            # set the control mode to current and set the current command to 0, i.e. no torque / no slack belt  
            self.left_boot.send_torque(0)           
            self.right_boot.send_torque(0)

    def free_training(self, restart_trial = False):
        """Free exploration training protocol, where the user walks with no guidance or feedback with a fixed Zhang/Collins controller.

        Parameters
        ----------
        restart_trial: boolean indicating if the trial should be restarted (default=False)

        Output
        ----------
        trial_running: indicates whether trial is running or ended (True=running, False=ended)
        """
        segment_min = 10                                                    # Free exploration training: 10 minutes of walking with fixed Collins controller 
        feedback_mode = -1                                                  # No feedback provided 
        use_torque = 1                                                      # Apply torque according to Zhang/Collins profile 
        trial_running = True                                                # Return value to let the external script know if the trial is over.
        current_idx = self.check_time(segment_min, restart_trial)           # Check if which trial segment we are in or if the trial has ended (-1)

        if restart_trial:
            self.left_gait.clear_gait_estimate()
            self.right_gait.clear_gait_estimate()
        
        if current_idx != -1:                                           
            self.trial_handler(feedback_mode[current_idx], use_torque[current_idx])
        else:
            # trial has ended, set current to zero and return that trial has finished 
            self.left_boot.send_torque(0)           
            self.right_boot.send_torque(0)
            trial_running = False
            
        return trial_running
    
    def guided_training(self, restart_trial = False):
        """Guided exploration training protocol, where the user walks with the fixed Zhang/Collins controller and audio feedback (metronome for 5 minutes, strategy feedback for 5 minutes).

        Parameters
        ----------
        restart_trial: boolean indicating if the trial should be restarted (default=False)

        Output
        ----------
        trial_running: indicates whether trial is running or ended (True=running, False=ended)
        """

        segment_min = [5, 5]                                                # Guided exploration training: 5 mins of metronome, 5 mins of strategy feedback
        feedback_mode = [0, 1]                                              # Metronome audio (0), then strategy feedback (1)
        use_torque = [1, 1]                                                 # Apply torque according to Zhang/Collins profile 
        trial_running = True                                                # Return value to let the external script know if the trial is over.
        current_idx = self.check_time(segment_min, restart_trial)           # Check if which trial segment we are in or if the trial has ended (-1)

        if restart_trial:
            self.left_gait.clear_gait_estimate()
            self.right_giat.clear_gait_estimate()

        if current_idx != -1:                                           
            self.trial_handler(feedback_mode[current_idx], use_torque[current_idx])
        else:
            # trial has ended, set current to zero and return that trial has finished 
            self.left_boot.send_torque(0)           
            self.right_boot.send_torque(0)
            trial_running = False
            
        return trial_running
    
    def testing(self, restart_trial = False):
        """Testing protocol, where the user walks with the fixed Zhang/Collins controller for 30 minutes. 

        Parameters
        ----------
        restart_trial: boolean indicating if the trial should be restarted (default=False)

        Output
        ----------
        trial_running: indicates whether trial is running or ended (True=running, False=ended)
        """
        segment_min = 30                                                    # 30 minutes of walking with Zhang/Collins controller 
        feedback_mode = -1                                                  # No feedback
        use_torque = 1                                                      # Apply torque according to Zhang/Collins profile 
        trial_running = True                                                # Return value to let the external script know if the trial is over.
        current_idx = self.check_time(segment_min, restart_trial)           # Check if which trial segment we are in or if the trial has ended (-1)

        if restart_trial:
            self.left_gait.clear_gait_estimate()
            self.right_gait.clear_gait_estimate()

        if current_idx != -1:                                           
            self.trial_handler(feedback_mode[current_idx], use_torque[current_idx])
        else:
            # trial has ended, set current to zero and return that trial has finished 
            self.left_boot.send_torque(0)           
            self.right_boot.send_torque(0)
            trial_running = False
            
        return trial_running
    