class GaitCalculator:
    """
    Parameters
    ----------
    _expected_stride_duration_ms : int
        The expected duration of a step in ms, based on the last few steps. Used to calculate the percent gait
    _expected_stance_duration_ms : int
        The expected duration of a stance in ms, based on the last few steps. Used to calculate the percent stance
    _heelstrike_trigger : bool
        True when heelstrike is detected, False otherwise
    _toeoff_trigger : bool
        True when toeoff is detected, False otherwise
    _heelstrike_armed : bool
        Used for a schmitt trigger for the heelstrike detector
    _heelstrike_armed_timestamp_ms : int
        Tracks when the heelstrike detector was armed
    _toeoff_trigger : bool 
        Used for a schmitt trigger for the heelstrike detector
    _toeoff_armed_timestamp_ms : int
        Tracks when the toeoff detector was armed

    _heelstrike_timestamp_current : int
        Most recent heelstrike timestamp
    _heelstrike_timestamp_previous : int
        Previous heelstrike timestamp
    _toeoff_timestamp_current : int
        Most recent toeoff timestamp
    _toeoff_timestamp_previous : int
        Previous toeoff timestamp 
    _num_stride_times_to_avg : int
        Number of stride durations averaged to estimate expected_stride_duration_ms
    _past_stride_durations : list 
        List of previous stride durations used to estimate expected_stride_duration_ms
     _past_stance_durations : list 
        List of previous stance durations used to estimate expected_stance_duration_ms

    Input Parameters
    ----------------
    _heelstrike_segmentation_arm_threshold_rad_s : int 
        Threshold (rad/s) used to trigger heelstrike_armed
    _heelstrike_armed_duration_percent_gait : int
        Time that gyro reading should be above heelstrike threshold before the heelstrike_trigger becomes True 

    Functions
    ----------------    
    _check_for_heelstrike()
        Checks for a heelstrike and return true if one is detected, and false otherwise
    _check_for_toeoff()
        Checks for a toeoff and return true if one is detected, and false otherwise
    _calc_percent_gait()
        Calculates an estimate of the percent gait and returns that estimate.
    _calc_percent_stance()
        Calculates an estimate of the percent stance and returns that estimate.
    """
    def __init__(self, heelstrike_segmentation_arm_threshold_rad_s = 150, heelstrike_armed_duration_percent_gait = 10, 
                 num_strides_to_avg = 10):
        # might want to make this a separate class.
        self._expected_stride_duration_ms = -1
        self._expected_stance_duration_ms = -1
        
        self.heelstrike_trigger = False
        self.toeoff_trigger = False
        
        self.percent_gait = -1
        self.percent_stance = -1 
        
        self._heelstrike_segmentation_arm_threshold_rad_s = heelstrike_segmentation_arm_threshold_rad_s
        self._heelstrike_armed_duration_percent_gait = heelstrike_armed_duration_percent_gait
        self._num_strides_to_avg = num_strides_to_avg 
        self._past_stride_durations = [-1] * self._num_strides_to_avg
        self._past_stance_durations = [-1] * self._num_strides_to_avg
        
        self._heelstrike_armed = False
        self._toeoff_armed = False
        self._heelstrike_armed_timestamp_ms = -1
        self._toeoff_armed_timestamp_ms = -1
        
        self._heelstrike_timestamp_current = -1
        self._heelstrike_timestamp_previous = -1
        self._toeoff_timestamp_current = -1
        self._toeoff_timestamp_previous = -1
        
        
        
    def calculate_gait_parameters(self, timestamp_ms, gyro_rad_s):
        self._check_for_heelstrike(timestamp_ms, gyro_rad_s)
        self._check_for_toeoff(timestamp_ms, gyro_rad_s)
        self._calc_percent_gait(timestamp_ms)
        self._calc_percent_stance(timestamp_ms)
        
        return {
            'percent_gait' : self._percent_gait,
            'percent_stance' : self._percent_stance,
            'heelstrike_trigger' : self._heelstrike_trigger,
            'toeoff_trigger' : self._toeoff_trigger,
        }
    
    def clear_gait_estimate(self):
        self._expected_stance_duration_ms = -1
        self._expected_stride_duration_ms = -1
        self._past_stride_durations = -1 * self._num_strides_to_avg
    
    def _check_for_heelstrike(self, timestamp_ms, gyro_rad_s):
        """
        Detects heelstrike based on whether gyro_rad_s exceeds the heelstrike segmentation threshold. 

        Parameters
        ----------
        timestamp_ms : int
            Current timestamp
        gyro_rad_s : float
            Gyroscope data in rad/s (suggested gyro_z)
            
        Returns
        -------
        None
        """
        # the trigger on the inversion of the leg is one method.
        # can also use spikes in acceleration (X seems to be best candidate but may not work well for slower gaits with smaller impacts)
        # Other candidates also possible.
        triggered = False
        armed_time = 0
        if self._heelstrike_armed_timestamp_ms != -1 :
            armed_time = timestamp_ms - self._heelstrike_armed_timestamp_ms
        if ((not self._heelstrike_armed) and gyro_rad_s >= self._heelstrike_segmentation_arm_threshold_rad_s):
            self._heelstrike_armed = True
            self._heelstrike_armed_timestamp_ms = timestamp_ms
        if (self._heelstrike_armed and (gyro_rad_s <= self._heelstrike_segmentation_arm_threshold_rad_s)):
            self._heelstrike_armed = False
            self._heelstrike_armed_timestamp_ms = -1 
            if  (armed_time > self._heelstrike_armed_duration_percent_gait/100 * self._expected_stride_duration_ms):
                triggered = True
                # update heelstrike timestamps and expected stride duration 
                self._update_expected_stride_duration(timestamp_ms)
            
        self.heelstrike_trigger = triggered
        # return self.heelstrike_trigger
    
    def _check_for_toeoff(self, timestamp_ms, gyro_rad_s):
        """
        Detects toeoff using sign inversion of gyro_rad_s from positive to negative (or flipped according to convention). 

        Parameters
        ----------
        timestamp_ms : int
            Current timestamp
        gyro_rad_s : float
            Gyroscope data in rad/s (suggested gyro_z)
            
        Returns
        -------
        None
        """
        # Check for gyro sign change from positive to negative (or flipped? check sign convention)

        triggered = False
        armed_time = 0
        if self._toeoff_armed_timestamp_ms != -1 :
            armed_time = timestamp_ms - self._toeoff_armed_timestamp_ms
        if ((not self._toeoff_armed) and (gyro_rad_s <= 0)):
            self._toeoff_armed = True
            self._toeoff_armed_timestamp_ms = timestamp_ms
        if (self._toeoff_armed and (gyro_rad_s > 0)):
            self._toeoff_armed = False
            self._toeoff_armed_timestamp_ms = -1
            if (armed_time > self._heelstrike_armed_duration_percent_gait/100 * self._expected_stride_duration_ms):
                triggered = True
                # update toeoff timestamps and expected stance duration 
                self._update_expected_stance_duration(timestamp_ms)
            
        self.toeoff_trigger = triggered
        # return self.toeoff_trigger
    
    def _update_expected_stride_duration(self, timestamp_ms):
        if (self._heelstrike_timestamp_previous == -1):
            self._heelstrike_timestamp_previous = timestamp_ms  # store the first heelstrike timestamp 
            return 
        
        self._heelstrike_timestamp_previous = self._heelstrike_timestamp_current
        self._heelstrike_timestamp_current = timestamp_ms
        stride_duration = self._heelstrike_timestamp_current - self._heelstrike_timestamp_previous

        if (-1 in self._past_stride_durations):
            self._past_stride_durations.insert(0, stride_duration)
            self._past_stride_durations.pop()
        elif (stride_duration >= 0.25 * min(self._past_stride_durations) and
              stride_duration <= 1.75 * min(self._past_stride_durations)): # check the stride duration is a reasonable length 
            self._past_stride_durations.insert(0, stride_duration)
            self._past_stride_durations.pop()
            # calculate average of previous stride durations and update expected stride duration 
            self._expected_stride_duration_ms = sum(self._past_stride_durations) / self._num_strides_to_avg
    
    def _update_expected_stance_duration(self, timestamp_ms):
        if (self._toeoff_timestamp_previous == -1):
            self._toeoff_timestamp_previous = timestamp_ms
            return
        
        self._toeoff_timestamp_previous = self._toeoff_timestamp_current
        self._toeoff_timestamp_current = timestamp_ms
        stance_duration = self._toeoff_timestamp_current - self._heelstrike_timestamp_current

        if (-1 in self._past_stance_durations):
            self._past_stance_durations.insert(0, stance_duration)
            self._past_stance_durations.pop()
        elif (stance_duration >= 0.25 * min(self._past_stance_durations) and 
              stance_duration >= 1.75 * max(self._past_stance_durations)):
            self._past_stance_durations.insert(0, stance_duration)
            self._past_stance_durations.pop()

            self._expected_stance_duration_ms = sum(self._past_stance_durations) / self._num_strides_to_avg

    def _calc_percent_gait(self, timestamp_ms):
        time_elapsed = timestamp_ms - self._heelstrike_timestamp_current
        self.percent_gait = time_elapsed / self._expected_stride_duration_ms * 100
        # cap percent_gait at 100 
        self.percent_gait = max(100, self.percent_gait) 

    def _calc_percent_stance(self, timestamp_ms):
        time_elapsed = timestamp_ms - self._heelstrike_timestamp_current
        self.percent_stance = time_elapsed / self._expected_stance_duration_ms * 100
        # if percent stance is >100, we are in swing, indicated by -1
        self.percent_stance = -1 if (self.percent_stance > 100) else self.percent_stance 