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
        Number of step durations averaged to estimate expected_step_duration_ms
    _past_stride_durations : list 
        List of previous step durations used to estimate expected_step_duration_ms

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
        
        self._heelstrike_armed = False
        self._heelstrike_armed_timestamp_ms = -1
        self._toeoff_armed_timestamp_ms = -1
        
        self._heelstrike_timestamp_current = -1
        self._heelstrike_timestamp_previous = -1
        self._toeoff_timestamp_current = -1
        self._toeoff_timestamp_previous = -1
        
        
        
    def calculate_gait_parameters(self, timestamp_ms, gyro_rad_s):
        self._check_for_heelstrike(timestamp_ms, gyro_rad_s)
        self._check_for_toeoff(timestamp_ms, gyro_rad_s)
        self._calc_percent_gait()
        self._calc_percent_stance()
        
        return {
            'percent_gait' : self._percent_gait,
            'percent_stance' : self._percent_stance,
            'heelstrike_trigger' : self._heelstrike_trigger,
            'toeoff_trigger' : self._toeoff_trigger,
        }
    
    def _check_for_heelstrike(self, timestamp_ms, gyro_rad_s):
        """
        DESCRIPTION
        TODO : see if there are any updates we want to make to be compatible with toeoff check.

        Parameters
        ----------
        None   
            
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
        if ((not self.heelstrike_armed) and gyro_rad_s >= self._heelstrike_segmentation_arm_threshold_rad_s) :
            self._heelstrike_armed = True
            self._heelstrike_armed_timestamp_ms = timestamp_ms
        if (self._heelstrike_armed and (gyro_rad_s <= self._heelstrike_segmentation_arm_threshold_rad_s)) :
            self.heelstrike_armed = False
            self._heelstrike_armed_timestamp_ms = -1 
            if  (armed_time > self._heelstrike_armed_duration_percent_gait/100 * self._expected_step_duration_ms) :
                triggered = True
                # update heelstrike timestamps and expected step duration 
                self._update_expected_duration(timestamp_ms)
            
        self.heelstrike_trigger = triggered
        return self.heelstrike_trigger
        
    
    def _check_for_toeoff(self, gyro_rad_s):
        """
        DESCRIPTION
        TODO : Need to figure out algorithm to use.

        Parameters
        ----------
        None   
            
        Returns
        -------
        None
        """
        # the trigger on the inversion of the leg is one method.
        # can also use spikes in acceleration (X seems to be best candidate but may not work well for slower gaits with smaller impacts)
        # Other candidates also possible.
        # triggered = False
        # armed_time = 0
        # if self.armed_timestamp != -1 :
            # armed_time = self.data_current[self.idx_time] - self.armed_timestamp
        # if ((not self.heelstrike_armed) and self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) :
            # self.heelstrike_armed = True
            # self.armed_timestamp = self.data_current[self.idx_time]
        # if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) ) :
            # self.heelstrike_armed = False
            # self.armed_timestamp = -1
            # if  (armed_time > ARMED_DURATION_PERCENT/100 * self.expected_duration) :
                # triggered = True
            
            
        # self.segmentation_trigger = triggered
        return -1
    
    def _update_expected_duration(self, timestamp_ms):

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
    
    def _calc_percent_gait(self, timestamp_ms):
        pass

    def _calc_percent_stance(self, timestamp_ms):
        pass