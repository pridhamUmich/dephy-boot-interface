import math as m
from flexsea.device import Device
import configparser
from serial.tools import list_ports
import json
import time
import numpy as np 
import matplotlib.pyplot as plt

# P. Pridham, May 2023; Modified by M. Wu, Sept 2023

class Exo : 
    """ 
    A class used to interface with the dephy exo.  Sends torque commands to the exo, and reads and stores data from the exo.

    ...

    Attributes
    ----------
    boot_firmware_version : string
        Stores the boot firmware version from the config file
    streaming_frequency : int
        Stores the streaming frequency from the config file.
    _device : actpack device
        Stores the actpack device that comes from the Dephy library
    raw_data : dict
        The raw data from the boot.
    data : dict
        The data converted to "normal" units, e.g. rads, Nm, A, etc.
    torque_cmd_Nm : float
        The most recent torque requested for the exo.
    current_cmd_A : float
        The most recent current value sent to the exo.
    is_left : bool
        Left side when true
    id_hex : string
        The exo id in hexadecimal   
        
    _direction : int
        1 when torque and angle don't need directional modification, -1 when the direction needs inversion from the raw.
    _wm_wa_coeffs : dict 
        Stores the polynomial coefficients for the derivative of the motor angle with respect to the ankle angle.  These are pulled from the config file.
    _wm_wa : float
        Stores the current derivative of the motor angle with respect to the ankle angle.  This is used for calculating the required motor torque to get the desired angle torque.
    _kt : float
        Motor torque constant (Nm/A).
    _motor_slack_position : float 
        Stores the motor position in the most dorsiflexed position, this can be used to slack the cable to make it more transparent.
    
    Methods
    -------
    __init__(port, is_left):
        Initializes the exo connected to the port, and sets the side based on is_left.  The port and id can be found using scan_for_boots() and the side can be found using get_is_left(id).
    __del__():
        Stops the motor, stops streaming, and closes the Dephy device.
    set_parameters(parameters):
        Sets the parameters for the exo.  Currently just a place holder.
    send_torque(torque_cmd_Nm):
        Sends a current command to the motor based on the provided ankle torque command in Nm, and sets the current_torque_cmd_Nm to the value sent.
    read_data():
        Reads and returns the data from the exo.
    
    _Nm_to_A(torque_cmd_Nm):
        Calculates the motor current needed to provide the requested ankle torque. This is based on the ankle angle and the assumption that the belt is tensioned.
    _calc_wm_wa():
        Calculates the current derivative of the motor angle with respect to the ankle angle.  
    
    
    Static Methods
    -------
    scan_for_boots(ports)
        Scans the ports to see if any boots are attached, takes in an array of port strings, returns a dict with the port and hex boot id if they are found.
    get_is_left(id)
        Checks the config file for the id and returns true if the boot is left, false if it is not left, and None if the id is not found.
    print_data(data_dict)
        Prints the data you give, should work for raw or processed data, or any dict really.
    
    _get_direction_from_is_left(is_left)
        Returns a multiplier to apply to the current to have the motor in the correct direction, this may also apply to various other measurements as well.
    _process_data(raw_data)
        Converts the raw data to 'regular' units
    _accl_bit_to_m_s2(raw_reading) 
        Converts raw accelerometer reading to m/s^2 
        ACCL LED up is +z reading; Shank up is -y reading; toe up left_boot +x reading, right_boot -x reading
        Raw readings don't seem to conform to right hand rule
        left raw looking at outside with shank up:
                       y^                      -----
                        |                   __|     |
        towards medial zX--->x towards heel|________|
        right raw looking at outside with shank up:
                       y^                  -----
                        |                 |     |__
        towards medial zX--->x towards toe|________|
    _gyro_bit_to_rad_s(raw_reading)
        Converts raw gyro reading to rad/s
        Same axes as accelerometer
    _ticks_to_rad(raw_reading)
        Converts the raw encoder reading to rad, this is for the motor it looks like the ankle just needs a deg->rad conversion
    _find_parameters_wm_wa(time_s, joint_angle_rad, motor_angle_rad, motor_acceleration_rad_s2)
        Takes in time in seconds, ankle angle, motor angle, and motor acceleration measurements, and returns an array of the coefficients of a 5th order polynomial fit to the motor angle with respect to the joint angle. These can be used to calculate the derivative of the motor angle with respect to the ankle angle.  This is used to determine the mechanical advantage for calculating the required motor torque to get the desired ankle torque.
    _check_and_run_calibration(device)
        Takes in a Dephy device from the flexsea library.  Runs a calibration on the device and save the values to a config file.  Default file is bootConfig.ini.
    """
    
    # set PID values to send
    pid_val_current_control = {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": 0,
        "b": 0,
        "ff": 128,
    }

    def __init__(self, port, is_left): 
        """
        Constructs all the necessary attributes for the Exo.  Unset parameters are None.

        Parameters
        ----------
        port : string 
            An string containing the port to open.  These can be found using the static method scan_for_boots
        is_left : bool
            Is set to true for the left boot.  This determines how directionality is handled.
        
        Returns
        -------
        None   
        
        """
        config = configparser.ConfigParser()
        config.read('bootConfig.ini')
        
        self.boot_firmware_version = config['Boot']['Version']
        self.streaming_frequency = config.getint('Boot','Streaming Frequency')
        
        # parameters
        self._device = Device(port=port, firmwareVersion=self.boot_firmware_version)
        self._device.open()  
        self._device.start_streaming(self.streaming_frequency)
        self._device.set_gains(**(Exo.pid_val_current_control))
        self.id_hex = f"{self._device.id:X}"
        
        self.torque_cmd_Nm = 0
        self.current_cmd_A = 0
        self.is_left = is_left # None for unknown
        self._direction = Exo._get_direction_from_is_left(is_left)
        # read in the values for the fit curve of the derivitave of the motor angle with respect to the ankle angle
        self._wm_wa_coeffs = {
            'poly4' : config.getfloat(self.id_hex,'poly4'), 
            'poly3' : config.getfloat(self.id_hex,'poly3'), 
            'poly2' : config.getfloat(self.id_hex,'poly2'), 
            'poly1' : config.getfloat(self.id_hex,'poly1'),  
            'poly0' : config.getfloat(self.id_hex,'poly0')
        }
        # variable to store the value of the current derivitave
        self._wm_wa = 0 
        self._kt = .146 #Nm/A  .146 for EB504, EB45 is .048 add to config if using two different versions
        self._current_limit_A = 25.0
        
        self._motor_slack_position = -1
        
        self.raw_data = self._device.read()
        self.data = Exo._process_data(self.raw_data, self.is_left)
        
    def __del__(self):
        """
        Stops the motors and streaming and closes the Dephy device.

        Parameters
        ----------
        None
        
        Returns
        -------
        None   
        """
        self._device.stop_motor()
        self._device.stop_streaming()
        self._device.close()
        
    def set_parameters(self, **kwargs):
        """
        Sets the parameters for the exo.  Currently just a place holder.

        Parameters
        ----------
        Keyword Arguments:
        None
            
        Returns
        -------
        None   
        """
        pass
        
    def send_torque(self,torque_cmd_Nm):
        """
        Sends a current command to the motor based on the provided ankle torque command in Nm, and sets the current_torque_cmd_Nm to the value sent.

        Parameters
        ----------
        torque_cmd_Nm : float
            Desired ankle torque in Nm.
        
        Returns
        -------
        None   
        """
        requested_current_A = self._Nm_to_A(torque_cmd_Nm)
        current_with_saturation_A = requested_current_A if np.abs(requested_current_A) < self._current_limit_A else (np.sign(requested_current_A) * self._current_limit_A)
        self._device.command_motor_current(self._direction * int(1000*current_with_saturation_A))
        self.torque_cmd_Nm = torque_cmd_Nm
    
    def read_data(self): 
        """
        Reads the data from the device and performs calculations to transfer the raw data to the regular data.

        Parameters
        ----------
        None   
            
        Returns
        -------
        None
        """
        self.raw_data = self._device.read()
        self.data = Exo._process_data(self.raw_data, self.is_left)

    ############################
    # Methods for internal use #
    ############################
    def _Nm_to_A(self, torque_cmd_Nm):
        """
        Calculates the motor current needed to provide the requested ankle torque. This is based on the ankle angle and the assumption that the belt is tensioned.

        Parameters
        ----------
        torque_cmd_Nm : float
            Desired ankle torque in Nm.
        
        Returns
        -------
        current_cmd_A : float
            The required motor current needed to provide the ankle torque based on the ankle position.
        """
        # find the current rate of change of the motor with respect to the motor positions, at the current ankle angle.
        self._calc_wm_wa()
        self.current_cmd_A = (torque_cmd_Nm / self._wm_wa) / self._kt # convert the ankle torque to motor torque, then divide by the motor torque constant to get the required current.
        return self.current_cmd_A
            
    def _calc_wm_wa (self):
        """
        Calculates the derivative of the motor angle with respect to the ankle angle, at the current ankle position.  

        Parameters
        ----------
        None
        
        Returns
        -------
        None   
        """
        self._wm_wa = 5 * self.data['ankle_angle_rad'] ** 4 * self._wm_wa_coeffs["poly4"] + 4 * self.data['ankle_angle_rad'] ** 3 * self._wm_wa_coeffs["poly3"] + 3 * self.data['ankle_angle_rad'] ** 2 * self._wm_wa_coeffs["poly2"] + 2 * self.data['ankle_angle_rad'] * self._wm_wa_coeffs["poly1"] + self._wm_wa_coeffs["poly0"]
        self._wm_wa = 1 if self._wm_wa <= .5 else self._wm_wa  # safety check to keep it from getting too large.
    
    
    ##################
    # Static Methods #
    ##################
    def scan_for_boots(port = 'all', do_calibration_check = False):
        """
        Scans the serial ports to see if any boots are attached.
        If boots are attached 
        
        Parameters
        ----------
        Keyword Arguments:
        port : string array
            The port(s) to scan.  Default is 'all'.
        do_calibration_check : bool
            Runs a calibration check for each boot if True, otherwise skips this check.  You may want to skip it if running headless.  This check is primarily for when you are setting up new boots.
        
        Returns
        -------
        boot_info : dict
            A dictionary with key strings of port names where boots are attached, and values of strings of the boot id in hexadecimal
        """
        # has only been tested with just boots. Not sure what happens when the com port is connected to something else.
        
        # Read in config file
        config = configparser.ConfigParser()
        config.read('bootConfig.ini')
        # get the firmware version used to open and scan the boots.
        boot_firmware_version = config['Boot']['Version']
        
        if(port == 'all') :
            # get open ports 
            open_ports = list_ports.comports()
        else :
            # just check the port given
            open_ports = port
        
        # iterate over open ports and check if they are boots, if the are boots return the port and boot id.
        num_valid_boots = 0
        boot_info = {}
        for p in open_ports:
            port_name = p.device # get the port name used to open the specific serial port
            test_boot = Device(port=port_name, firmwareVersion=boot_firmware_version) # create a boot to get information for.
            
            try:
                test_boot.open()   
                test_boot.start_streaming(10) # use a low streaming frequency for calibration
                # if a calibration check is desired, do it
                if do_calibration_check:
                    #check and run calibration
                    Exo._check_and_run_calibration(test_boot)
                boot_info.update({port_name : f"{test_boot.id:X}"}) # package the port and id pairs to return
            except RuntimeError :
                print('Exo :: scan_for_boots : RuntimeError - likely there is a port that is not a boot')
            
        return boot_info
        
    def get_is_left(dev_id):
        """
        Checks the config file for a section with the device id and returns true if the boot is left, false if it is not left, and None if the id is not found.
        
        Parameters
        ----------
        device_id : string
            A string containing the hexadecimal id of the boot to check the side of.
        
        Returns
        -------
        Side : bool 
            True - boot id is in config file and the boot is on the left side.
            False - boot id is in config file and the boot is on the right side 
            None - boot id is not in the config or the side information doesn't exist.
        """
        # has only been tested with just boots. Not sure what happens when the com port is connected to something else.
        config = configparser.ConfigParser()
        config.read('bootConfig.ini')
        
        # check the sides return true for left, false for right, and none otherwise
        try :
            is_left = True if ('left' == config[dev_id]['Side']) else False if ('right' == config[dev_id]['Side']) else None
        # I think this might error out if the id does not exist so catch it and return None. 
        except :
            is_left = None
            
        return is_left

    def print_data(data):
        """
        prints the data you give, should work for raw or processed data, or any dict really.

        Parameters
        ----------
        data : dict
            Description
        
        Returns
        -------
        None   
        """
        pretty_dict = json.dumps(data, indent=4)
        print(pretty_dict)
    
    ###################################
    # Static Methods for internal use #
    ###################################
    def _get_direction_from_is_left(is_left):
        """
        Returns a multiplier to apply to the current to have the motor in the correct direction, this may also apply to various other measurements as well.  -1 when is_left is true and 1 otherwise.

        Parameters
        ----------
        is_left : bool   
            True if the boot is on the left side, false otherwise
        Returns
        -------
        multiplier : int
            A multiplier to apply to the current to have the motor in the correct direction, this may also apply to various other measurements as well.  -1 when is_left is true and 1 otherwise.
        """
        return -1 if is_left else 1 # this is just for the EB504 may need to move this to the config if we want compatibility with other versions.
    
    def _process_data(raw_data, is_left): 
        """
        Returns the data dict from the processed raw data only pulling the relevant fields.  This should only be called internally when initializing or in self.read_data()

        Parameters
        ----------
        raw_data : dict   
            The raw data dictonary from the dephy device.read()
        is_left : bool
            True if the boot is on the left side, false otherwise
        Returns
        -------
        data : dict
            The raw data converted to 'regular' units
        """
        # TODO : figure out units and make converters 
        direction = Exo._get_direction_from_is_left(is_left)
        data = {
            "state_time_ms" : raw_data["state_time"],
            "accl_x_m_s2" : Exo._accl_bit_to_m_s2(raw_data["accelx"]),
            "accl_y_m_s2" : Exo._accl_bit_to_m_s2(raw_data["accely"]),
            "accl_z_m_s2" : Exo._accl_bit_to_m_s2(raw_data["accelz"]),
            "gyro_x_rad_s" : Exo._gyro_bit_to_rad_s(raw_data["gyrox"]),
            "gyro_y_rad_s" : Exo._gyro_bit_to_rad_s(raw_data["gyroy"]),
            "gyro_z_rad_s" : Exo._gyro_bit_to_rad_s(raw_data["gyroz"]),
            "motor_angle_rad" : direction * Exo._ticks_to_rad(raw_data["mot_ang"]),
            "motor_velocity_rad_s" : direction * raw_data["mot_vel"]*m.pi/180,
            "motor_current_A" : direction * raw_data["mot_cur"]/1000,
            "ankle_angle_rad" : direction * Exo._ticks_to_rad(raw_data["ank_ang"]), #/100*m.pi/180, # the values appear to be in ticks rather than deg*10 like the documentation says, this is likely firmware specific so other versions would need a different conversion.
            "ankle_velocity_rad_s" : direction * raw_data["ank_vel"]/10*m.pi/180,
            "battery_voltage_V" : raw_data["batt_volt"]/1000,
        }
        return data
    
    def _accl_bit_to_m_s2(raw_reading):
        """
        Converts raw accelerometer reading to m/s^2 
        ACCL LED up is +z reading; Shank up is -y reading; toe up left_boot +x reading, right_boot -x reading
        Raw readings don't seem to conform to right hand rule
        left raw looking at outside with shank up:
                       y^                      -----
                        |                   __|     |
        towards medial zX--->x towards heel|________|
        right raw looking at outside with shank up:
                       y^                  -----
                        |                 |     |__
        towards medial zX--->x towards toe|________|
        Values from https://dephy.com/start/#units
        
        Parameters
        ----------
        raw_reading : int
            The raw reading from the accelerometer.
        
        Returns
        -------
        acceleration_m_s2 : float
            The acceleration reading converted to m/s^2
        """
        return raw_reading/8192*9.81 # raw/(LSB/g)*((m/s^2)/g)
        
    def _gyro_bit_to_rad_s(raw_reading):
        """
        Converts raw gyro reading to rad/s
        Values from https://dephy.com/start/#units
        
        Parameters
        ----------
        raw reading : int
            The raw reading from the gyro
        
        Returns
        -------
        acceleration_m_s2 : float
            The gyro reading converted to rad/s 
        """
        return raw_reading/32.8*m.pi/180 # raw/(LSB/(deg/s))*(rad/deg)
        
    def _ticks_to_rad(raw_reading):
        """
        Converts the raw encoder reading to rad, this is for the motor it looks like the ankle just needs a deg->rad conversion
        Values from https://dephy.com/start/#units
        
        Parameters
        ----------
        raw_reading : int
            raw encoder count for the motor.
        
        Returns
        -------
        encoder_angle_rad : float
            The encoder angle in radians
            
        TODO : Check with Dephy that 0.02197 is the correct multiplier.
        """
        return raw_reading*2*m.pi/16384 # raw*(2*pi/ticks_per_rotation)
        
    def _find_parameters_wm_wa(time_s, joint_angle_rad, motor_angle_rad, motor_acceleration_rad_s2, acceleration_threshold_rad_s2 = 200, make_plot = True, start_idx = 0):
        """
        Takes in time in seconds, ankle angle, motor angle, and motor acceleration measurements, and returns an array of the coefficients of a 5th order polynomial fit to the motor angle with respect to the joint angle. These can be used to calculate the derivative of the motor angle with respect to the ankle angle.  This is used to determine the mechanical advantage for calculating the required motor torque to get the desired ankle torque.
        
        Parameters
        ----------
        time_s : array type
            An array of timestamps for the other data in seconds.
        joint_angle_rad : array type
            The joint (ankle) angle measurements at each time point.
        motor_angle_rad : array type  
            The motor angle measurements at each time point.
        motor_acceleration_rad_s2 : array type
            The acceleration of the motor at each time point, this is used to determine when the belt becomes taught.
        
        Keyword Arguments:
        acceleration_threshold_rad_s2 : int or float
            A threshold the acceleration of the motor need to be below for a solid second before the data will be considered valid. Default is 200.
        make_plot : bool
            Flag used to determine if plots should be displayed, plots will be displayed when true.  Default is True.
        start_idx : int
            Some ways of collecting data may have some preceding data that isn't of interest.  This value can be used to skip over this data, as the algorithm will skip any values before start_idx.  Default is 0.
        
            
        Returns
        -------
        polynomial_coeff : Array type, Floats 
            An array of the coefficients of a 5th order polynomial fit to the motor angle with respect to the joint angle. These can be used to calculate the derivative of the motor angle with respect to the ankle angle.  This is used to determine the mechanical advantage for calculating the required motor torque to get the desired ankle torque.
        """
        # look for time when the next second is below the acceleration threshold
        
        # find the index count corresponding to 1 second
        timestep_s = np.mean(np.diff(time_s)) 
        num_idx_one_second = m.ceil(1/timestep_s)
        print(f'Exo :: _find_parameters_wm_wa : num_idx_one_second = {num_idx_one_second}')
        
        if make_plot:
            fig, ax = plt.subplots()
            ax.scatter(joint_angle_rad, motor_angle_rad) # show the data used to fit
            ax.set(xlabel='Ankle Angle (rad)', ylabel='Motor Angle (rad)',
            title='Full\nAnkle vs Motor Angle')
            plt.show()
            
        # look at sections in the code till you find a point where the motor acceleration is continually below the threshold, this should indicate the slack is pulled up. This will be the start of the data to use.  This was needed for older boots that had hard stops.  Newer boots the strap is taught at full extension.
        for i in range(start_idx,time_s.size - 1 - num_idx_one_second): # -1 needed due to zero indexing
            if (num_idx_one_second == sum(np.array(motor_acceleration_rad_s2)[i:(i+num_idx_one_second+1)] < acceleration_threshold_rad_s2)): # plus one due to how python slicing works in python
                start_idx = i
                break
        # find where the joint angle reaches its max value, this is the end of the data to use to find the parameters.
        print(f'Exo :: _find_parameters_wm_wa : start_idx = {start_idx}')
        stop_idx = 1+motor_angle_rad.index(np.amax(motor_angle_rad))
        print(f'Exo :: _find_parameters_wm_wa : stop_idx = {stop_idx}')
        # pull out the data in the range we care about
        ankle_of_interest = joint_angle_rad[start_idx:stop_idx]
        motor_of_interest = motor_angle_rad[start_idx:stop_idx]
        if make_plot:
            fig, ax = plt.subplots()
            ax.scatter(ankle_of_interest, motor_of_interest) # show the data used to fit
            ax.set(xlabel='Ankle Angle (rad)', ylabel='Motor Angle (rad)',
            title='Ankle vs Motor Angle\nIn range of interest')
            plt.show()
        
        # find the unique ankle angle values, and corresponding motor values.  I think one of the ways we did this originally needed this to be monotonic, it may not be needed now.
        ankle_values, ankle_idx = np.unique(ankle_of_interest, return_index = True)
        
        motor_values = np.array(motor_of_interest)[ankle_idx]
        # make the plots if desired
        if make_plot:
            fig, ax = plt.subplots()
            ax.scatter(ankle_values, motor_values) # show the data used to fit
            ax.set(xlabel='Ankle Angle (rad)', ylabel='Motor Angle (rad)',
            title='Ankle vs Motor Angle\nUnique Ankle Values')
            plt.show()
        # need to use the new_series as an intermediate to pull out the coefficients from the fit.    
        new_series = np.polynomial.Polynomial.fit(ankle_values, motor_values, 5)
        polynomial_coeff = new_series.convert().coef  # get the coefficients of the polynomial.
        
        # make the plots if desired
        if make_plot:
            fig, ax = plt.subplots()
            ax.scatter(ankle_values, motor_values) # show the data used to fit
            # plot the fit curve
            ax.plot(ankle_values, ankle_values ** 5 * polynomial_coeff[5] + ankle_values ** 4 * polynomial_coeff[4] + ankle_values ** 3 * polynomial_coeff[3] + ankle_values ** 2 * polynomial_coeff[2]+ ankle_values * polynomial_coeff[1] + polynomial_coeff[0], 'r')
            ax.set(xlabel='Ankle Angle (rad)', ylabel='Motor Angle (rad)',
            title='Ankle vs Motor Angle\nWith Fit')
            plt.show()
            
            # check the derivative of the fit
            fig, ax = plt.subplots()
            ax.plot(ankle_values, 5 * ankle_values ** 4 * polynomial_coeff[5] + 4 * ankle_values ** 3 * polynomial_coeff[4] + 3 * ankle_values ** 2 * polynomial_coeff[3] + 2 * ankle_values * polynomial_coeff[2]+ polynomial_coeff[1])
            ax.set(xlabel='Ankle Angle (rad)', ylabel='dMotor/dAnkle',
            title='Ankle vs dMotor/dAnkle')
            plt.show()
            
        return polynomial_coeff[1:] # only return the ones used for the derivative
    
    def _check_and_run_calibration(device, config_filename = 'bootConfig.ini', current_cmd_A = .5):
        """
        Takes in a Dephy device from the flexsea library.  Runs a calibration on the device and save the values to a config file.  Default file is bootConfig.ini.
        
        Parameters
        ----------
        device : flexsea device
            A flexsea device used interface with a dephy exoboot.
        
        Keyword Arguments:
        config_filename : string
            The config file to read and write to.  Default is 'bootConfig.ini'
        current_cmd_A : float
            The current to apply to the exo during the calibration. Default is .5
        
        Returns
        -------
        None
        """
        # read in the config file
        config = configparser.ConfigParser()
        config.read(config_filename)
        
        do_calibration = False  # set flag for checks if a new calibration should be done
        id_hex = f"{device.id:X}"  # get the hex id of the device to find in the config file
        
        # check if boot has a section in config
        if config.has_section(id_hex):
            # if a parameter is unset then require a calibration
            if ((config.getfloat(id_hex,'poly4') == -1) 
                or (config.getfloat(id_hex, 'poly3') == -1) 
                or (config.getfloat(id_hex, 'poly2') == -1)
                or (config.getfloat(id_hex, 'poly1') == -1)
                or (config.getfloat(id_hex, 'poly0') == -1)
                or not((config[id_hex]['Side'] == 'left') or (config[id_hex]['Side'] == 'right'))): # nor so it if either is value is there
                do_calibration = True
                print('One or more keys have invalid values, calibration is required')
            # Print the values for the device so a person can decide if a new calibration is desired.
            print('id_hex')    
            pretty_dict = json.dumps(dict(config.items(id_hex)), indent=4)
            print(pretty_dict)
            response = '' # place holder to catch the keyboard input to check if it is a valid response.
            valid_input = False
            # if we don't have to do a calibration check if they would like to
            if not do_calibration: 
                while not valid_input:
                    response = input('Would you like to calibrate (y/n): ')
                    if response == 'y' or response == 'n': 
                        valid_input = True
                    else :
                        print('Invalid input, please try again')
                if response == 'y':
                    do_calibration = True
        else : #boot has no calibration section so require calibration
            print('No boot id not found in config file, calibration required')
            do_calibration = True
        if do_calibration:
            valid_input = False
            while not valid_input:
                    side = input(f'Select side for {id_hex} (l/r) : ')
                    if side == 'l' or side == 'r': 
                        valid_input = True
                    else:
                        print('Invalid input, please try again')
            
            is_left = True if side == 'l' else False
            direction = Exo._get_direction_from_is_left(is_left)
            # create arrays to store data for calibration.
            time_ms = []
            motor_angle_rad = []
            motor_acceleration_rad_s2 = []
            ankle_angle_rad = []
            # run till keyboard interrupt.
            try:
                trash = input('Prepare to collect data. Hold shank as far as you can towards the toe.  A small force will be applied.  Once the belt is tight slowly move the shank towards the heel making sure the belt stays tight. \n\nPress Enter to start.  \nPress ctrl+c when done.')
                time.sleep(1) # give a second for the person to get their hands back.
                
                device.set_gains(**(Exo.pid_val_current_control))
                while do_calibration :
                    # read the data and pull out the values needed to find the coefficients
                    # print('Exo :: _check_and_run_calibration() : In while loop')
                    raw_data = device.read()
                    # print('Exo :: _check_and_run_calibration() : device.read done')
                    data = Exo._process_data(raw_data, is_left)
                    # print('Exo :: _check_and_run_calibration() : Exo._process_data done')
                    # Exo.print_data([raw_data, data])
                    time_ms.append(data['state_time_ms'])
                    motor_acceleration_rad_s2.append(direction * raw_data['mot_acc'])
                    motor_angle_rad.append(data['motor_angle_rad'])
                    ankle_angle_rad.append(data['ankle_angle_rad'])
                    device.command_motor_current(int(direction * current_cmd_A * 1000))
                    
            except KeyboardInterrupt:
                device.stop_motor()
            except RuntimeError :
                print('-- Exo :: _check_and_run_calibration() : RuntimeError - device might not be streaming yet.')
                return
            
            print('\n\nData collection finished.  It may take some time. Please be patient.')
            
            # calculate parameters 
            polynomial_coeff = Exo._find_parameters_wm_wa((np.array(time_ms)-time_ms[0])/1000, ankle_angle_rad, motor_angle_rad, motor_acceleration_rad_s2)
            print(f'-- Exo :: _check_and_run_calibration : polynomial_coeff = {polynomial_coeff}')
            # Check if boot section exists and add if it doesn't update if it does.
            if not config.has_section(id_hex):
                config.add_section(id_hex)
            # create keys or overwrite them if they already exist.
            config[id_hex]['side'] = 'left' if is_left else 'right'
            config[id_hex]['poly4'] = f'{polynomial_coeff[4]}'
            config[id_hex]['poly3'] = f'{polynomial_coeff[3]}'
            config[id_hex]['poly2'] = f'{polynomial_coeff[2]}'
            config[id_hex]['poly1'] = f'{polynomial_coeff[1]}'
            config[id_hex]['poly0'] = f'{polynomial_coeff[0]}'
            
            # Write the updated config file.
            file = open(config_filename, 'w')
            config.write(file)# need to check if you need a file object open
            file.close()
    
if __name__ == '__main__':
    import os
    from DataLogger import *
    
    # get the boots that are attached and ask the person if they would like to calibrate the boots, or run the calibration if needed.
    available_boots = Exo.scan_for_boots(do_calibration_check = True)
    # from the list of available boots attach them to the left or right boots, assumes one of each.
    for b in available_boots:
        is_left = Exo.get_is_left(available_boots[b])
        if is_left :
            left_boot = Exo(b, is_left)
        elif None != is_left :
            right_boot = Exo(b, is_left)
            
    try:
        if 'left_boot' in locals():
            left_data_logger = DataLogger([left_boot.raw_data, left_boot.data], path = 'test_folder', base_name = f'left_{left_boot.id_hex}')
        if 'right_boot' in locals():
            right_data_logger = DataLogger([right_boot.raw_data, right_boot.data], path = 'test_folder', base_name = f'right_{right_boot.id_hex}')
        
        while True :
            os.system('cls')  # clear the screen so we are only showing the most recent reading.
            if 'left_boot' in locals():
                left_boot.read_data()
                print('\nLeft')
                Exo.print_data(left_boot.raw_data)
                Exo.print_data(left_boot.data)
                left_boot.send_torque(.25)
                # left_boot._device.command_motor_current(int(500)) # for use when we don't have the calibration done yet.
                left_data_logger.log([left_boot.raw_data, left_boot.data])
                
            if 'right_boot' in locals():
                right_boot.read_data()
                print('\nRight')
                Exo.print_data(right_boot.raw_data)
                Exo.print_data(right_boot.data)
                right_boot.send_torque(.25)
                # right_boot._device.command_motor_current(int(500))  # for use when we don't have the calibration done yet.
                right_data_logger.log([right_boot.raw_data, right_boot.data])
                
            if (('left_boot' not in locals()) and ('right_boot' not in locals())):
                print('\nNo Exos found, please connect and turn on then restart program.')
                
           
    except KeyboardInterrupt:
        pass
        
        ##ACCL LED up is +z reading, Shank up is -y reading, toe up left_boot +reading right_boot - reading
        ## ankle ang, right decreases with plantarflexion, not zeroed left increases with plantar flexion
        ## Motor command Left positive wraps underhand  right positive wraps overhand
       ## motor angle led up counter clockwise is positive.