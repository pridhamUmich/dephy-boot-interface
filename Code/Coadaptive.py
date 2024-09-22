from Controller import *
import math as m 
import numpy as np 

# M. Wu, September 2023

class Coadaptive(Controller):
    """
    A class to represent a co-adaptive controller, which modulates the peak torque provided based on EMG, IMU, and ankle kinematics.
    The torque profile is based on the Zhang/Collins profile from: 
        Zhang, J., Fiers, P., Witte, K. A., Jackson, R. W., Poggensee, K. L., Atkeson, C. G., & Collins, S. H. (2017). 
        Human-in-the-loop optimization of exoskeleton assistance during walking. Science, 356(6344), 1280-1284.

    Attributes
    ----------
    t0 : float
        Ramp start percent
    t1 : float
        Ramp end percent
    t2 : float
        Peak percent
    t3 : float
        Stop percent
    ts : float
        Top torque for the ramp
    tp : float
        Peak torque (Nm)
    user_mass : float
        User mass (kg)
    peak_torque_normalized : float
        Peak torque normalized to user mass
    a1 : float
        Parameter for the Zhang/Collins torque curve rising section : a1 * percent_gait^3 + b1 * self.percent_gait^2 + c1 * percent_gait + d1
    b1 : float
        Parameter for the Zhang/Collins torque curve rising section : a1 * percent_gait^3 + b1 * self.percent_gait^2 + c1 * percent_gait + d1
    c1 : float
        Parameter for the Zhang/Collins torque curve rising section : a1 * percent_gait^3 + b1 * self.percent_gait^2 + c1 * percent_gait + d1
    d1 : float
        Parameter for the Zhang/Collins torque curve rising section : a1 * percent_gait^3 + b1 * self.percent_gait^2 + c1 * percent_gait + d1
        
    a2 : float
        Parameter for the Zhang/Collins torque curve falling section : a2 * percent_gait^3 + b2 * self.percent_gait^2 + c2 * percent_gait + d2
    b2 : float
        Parameter for the Zhang/Collins torque curve falling section : a2 * percent_gait^3 + b2 * self.percent_gait^2 + c2 * percent_gait + d2
    c2 : float
        Parameter for the Zhang/Collins torque curve falling section : a2 * percent_gait^3 + b2 * self.percent_gait^2 + c2 * percent_gait + d2
    d2 : float
        Parameter for the Zhang/Collins torque curve falling section : a2 * percent_gait^3 + b2 * self.percent_gait^2 + c2 * percent_gait + d2
        
    percent_gait : float
        The most recent percent gait used to calculate the torque

    Methods
    -------
    set_parameters(parameters):
        Calculates the spline parameters a-d for the curve so we don't need to redo this every loop.  Should be called when the nature of the curve changes.  If all the curve parameters are not set it will not recalculate.
        
    calculate_torque_cmd(state_info):
        Calculates the current torque command based on the supplied state info, the percent gait.  If the state info is not set correctly it will output 0.
    """

    def __init__(self) : 
        """
        Constructs all the necessary attributes for the ZhangCollins Controller.  Unset parameters are -1.

        Parameters
        ----------
        None
            
        Returns
        -------
        None   
        
        """
        super().__init__()
        
        # Zhang/Collins parameters
        self.t0 = -1        # ramp start percent
        self.t1 = -1        # ramp end percent
        self.t2 = -1        # peak percent
        self.t3 = -1        # drop percent
        self.ts = -1        # top torque for the ramp
        self.tp = -1        # peak torque
        self.user_mass = -1    # user mass
        self.peak_torque_normalized = -1    # torque normalized to user mass
        
        # parameters for the Zhang/Collins torque curve
        self.a1 = -1
        self.b1 = -1
        self.c1 = -1
        self.d1 = -1
        
        self.a2 = -1
        self.b2 = -1
        self.c2 = -1
        self.d2 = -1
        
        self.percent_gait = -1
    
    
    def set_parameters(self, **kwargs) :
        """
        Calculates the spline parameters a-d for the curve so we don't need to redo this every loop.  Should be called when the nature of the curve changes.  If all the curve parameters are not set (!=-1) it will not recalculate.

        Parameters
        ----------
        Keyword Arguments:
            user_mass : float
                Mass in kg of the person wearing the system.
            ramp_start_percent_gait : float
                The percent gait to start tensioning the belt.
            onset_percent_gait : float
                The percent gait to start ramping up the torque.
            peak_percent_gait : float
                The percent gait the peak torque will occur.
            stop_percent_gait : float
                The percent gait torque will stop being applied.
            onset_torque : float
                Torque Nm the belt will tension to prior to the onset of the push off torque
            normalized_peak_torque : float
                Magnitude of the peak torque normalized to user mass (Nm/kg)
            
        Returns
        -------
        None   
        """
        
        # average values from the zhang/collins optimization paper.
        # t0 = 0;
        # t1 = 27.1;
        # t2 = 50.4;
        # t3 = 62.7;
        # ts = 2;

        # peakTorqueNormalized = 0.20; # 0.76; # Using a smaller value due to Dephy Exo Limit.
        
        # Check that the correct parameters came in
        if ("user_mass" in kwargs and kwargs["user_mass"] != -1) :
            self.user_mass = kwargs["user_mass"] # kg
            # print(f'ZhangCollins::set_parameters : self.user_mass = {self.user_mass}')
        if ("ramp_start_percent_gait" in kwargs and kwargs["ramp_start_percent_gait"] != -1) :
            self.t0 = kwargs["ramp_start_percent_gait"]
        if ("onset_percent_gait" in kwargs and kwargs["onset_percent_gait"] != -1) :
            self.t1 = kwargs["onset_percent_gait"]
        if ("peak_percent_gait" in kwargs and kwargs["peak_percent_gait"] != -1) :
            self.t2 = kwargs["peak_percent_gait"]
        if ("stop_percent_gait" in kwargs and kwargs["stop_percent_gait"] != -1) :
            self.t3 = kwargs["stop_percent_gait"]
        if ("onset_torque" in kwargs and kwargs["onset_torque"] != -1) :
            self.ts = kwargs["onset_torque"]
        if ("normalized_peak_torque" in kwargs and kwargs["normalized_peak_torque"] != -1) :
            self.peak_torque_normalized = kwargs["normalized_peak_torque"]
        
        # if all the parameters are set calculate the spline parameters
        if (self.user_mass != -1 and self.t0  != -1, self.t1  != -1 and self.t2  != -1 and self.t3  != -1 and self.ts  != -1 and self.peak_torque_normalized  != -1) :
            
            self.tp = self.user_mass * self.peak_torque_normalized;
            
            self.a1 = (2 *(self.tp - self.ts))/m.pow((self.t1 - self.t2),3);
            self.b1 = -((3 *(self.t1 + self.t2) *(self.tp - self.ts)) / m.pow((self.t1 - self.t2),3));
            self.c1 = (6* self.t1 * self.t2 * (self.tp - self.ts))/ m.pow((self.t1 - self.t2),3);
            self.d1 = -((-m.pow(self.t1, 3) *self.tp + 3 * m.pow(self.t1, 2)* self.t2 * self.tp - 3 * self.t1 * m.pow(self.t2,2) * self.ts + m.pow(self.t2,3) * self.ts)/ m.pow((self.t1 - self.t2),3));

            
            self.a2 = -((self.tp - self.ts)/(2* m.pow((self.t2 - self.t3),3)));
            self.b2 = (3 *self.t3 *(self.tp - self.ts))/(2 * m.pow((self.t2 - self.t3),3));
            self.c2 = (3 *(m.pow(self.t2,2) - 2 *self.t2 *self.t3) * (self.tp - self.ts))/(2* m.pow((self.t2 - self.t3),3));
            self.d2 = -((3 * m.pow(self.t2,2) * self.t3 * self.tp - 6 * self.t2 * m.pow(self.t3, 2) * self.tp + 2 * m.pow(self.t3,3) * self.tp - 2 * m.pow(self.t2,3) * self.ts + 3 * m.pow(self.t2, 2) * self.t3 * self.ts)/(2 * m.pow((self.t2 - self.t3), 3)));
        # if not everything is set print the inputs so the user can see what is not set.    
        else :
            print('ZhangCollins :: set_parameters : one of the parameters is not set' + \
                '\n user_mass : ' + str(self.user_mass) + \
                '\n ramp_start_percent_gait : ' + str(self.t0) + \
                '\n onset_percent_gait : ' + str(self.t1) + \
                '\n peak_percent_gait : ' + str (self.t2) + \
                '\n stop_percent_gait : ' + str (self.t3) + \
                '\n onset_torque : ' + str (self.ts) + \
                '\n normalized_peak_torque : ' + str (self.peak_torque_normalized))
               
    def calculate_torque_cmd(self, **kwargs) : 
        """
        Calculates the current torque command based on the supplied state info, the percent gait.  If the state info is not set correctly it will output 0.

        Parameters
        ----------
        Keyword Arguments:
            percent_gait : float
                The percent of gait the torque should be calculated for.    
            
        Returns
        -------
        torque_cmd : float  
            The calculated torque command in Nm.
        """
        
        # Check that the state info that came in is correct.
        if ("percent_gait" in kwargs and kwargs["percent_gait"] != -1) :
            self.percent_gait = kwargs["percent_gait"]
        # If something is not set print the needed parameters so the user can see what is not set.
        else :
            print('ZhangCollins :: calculate_torque_cmd : some of the data is not set' + \
                '\n percent_gait : ' + str(self.percent_gait))
        tau = 0 # the torque command, set to zero so if we don't make a change it will output 0.
        
        
        if (self.percent_gait != -1) : # check that the percent gait is set
            if ((self.percent_gait <= self.t1)  and  (self.t0 <= self.percent_gait)) : # torque ramp to ts at t1
                tau = self.ts / (self.t1 - self.t0) * self.percent_gait - self.ts/(self.t1 - self.t0) * self.t0;
            
            elif (self.percent_gait <= self.t2) : # the rising spline
                tau = self.a1 * m.pow(self.percent_gait,3) + self.b1 * m.pow(self.percent_gait,2) + self.c1 * self.percent_gait + self.d1;
                
            elif (self.percent_gait <= self.t3) : # the falling spline
               tau = self.a2 * m.pow(self.percent_gait,3) + self.b2 * m.pow(self.percent_gait,2) + self.c2 * self.percent_gait + self.d2;
                
            else : # go to the slack position if we aren't commanding a specific value
                tau = 0;
                
        
        self.torque_cmd = tau # store the torque command.
        return tau # return the torque command.

    def update_parameters(self, **kwargs):
        """
        Updates the torque profile based on EMG, IMU, and ankle data from Dephy exoskeleton. 

        Parameters
        ----------
        EMG_data: readings from electromyography sensors for muscle activity data (up to 16 sensors, passed from EMG module)
        IMU: readings from inertial measurement unit sensors for angular kinematics 
        ankle: readings from Dephy ankle exoskeleton  
        """
        k1 = 0.1
        k2 = 0.1
        k3 = 0.11
        k4 = 0.04
        k5 = 0.04
        k6 = 0.04
        k7 = 0.06
        k8 = 0.05
        k9 = 0.00845
        m_ankle = 1.3
        m_hip = 1.0
       
        """
        EMG Readings - Order of Sensors + Muscles must be consistent

        Order of Sensors - SOL, GAS, TA, BF, RF, TFL, Gmax 
                           0    1    2   3   4   5    6
        """
        
        if("emg_data" in kwargs):
            """
            Ankle Contributions 
            """
            #SOL 
            sol_data = kwargs["emg_data"][0] / np.power(self.mean_time_series(self, kwargs["emg_data"][0]), m_ankle)
            sol_change = self.calculate_rms(sol_data)
            tp = tp + k1 * sol_change

            #GAS 
            gas_data = kwargs["emg_data"][1] / pow(self.mean_time_series(self, kwargs["EMG_data"][1]), m_ankle)
            gas_change = self.calculate_rms(gas_data)
            tp = tp + k2 * gas_change

            #TA 
            ta_change = self.calculate_rms(self, kwargs["emg_data"][2])
            tp = tp - k3 * ta_change

            '''
            Knee Contributions
            To Do: 
            Calculate torque changes for knee CCI 
            '''
            
            '''
            Hip Contributions
            '''
            tfl_data = kwargs["emg_data"][5]
            tfl_change = self.calculate_rms(np.power(self.mean_time_series(tfl_data), m_hip))
            tp = tp + k6 * tfl_change

            '''
            To Do: 
            Calculate torque changes for hip CCI 
            '''

        if("imu_data" in kwargs):
            """
            To Do: 
            include torque changes from ankle and hip angles
            """
            

        #update the parameters now that tp has been changed 
        self.a1 = (2 *(self.tp - self.ts))/m.pow((self.t1 - self.t2),3)
        self.b1 = -((3 *(self.t1 + self.t2) *(self.tp - self.ts)) / m.pow((self.t1 - self.t2),3))
        self.c1 = (6* self.t1 * self.t2 * (self.tp - self.ts))/ m.pow((self.t1 - self.t2),3)
        self.d1 = -((-m.pow(self.t1, 3) *self.tp + 3 * m.pow(self.t1, 2)* self.t2 * self.tp - 3 * self.t1 * m.pow(self.t2,2) * self.ts + m.pow(self.t2,3) * self.ts)/ m.pow((self.t1 - self.t2),3))
        
        self.a2 = -((self.tp - self.ts)/(2* m.pow((self.t2 - self.t3),3)))
        self.b2 = (3 *self.t3 *(self.tp - self.ts))/(2 * m.pow((self.t2 - self.t3),3))
        self.c2 = (3 *(m.pow(self.t2,2) - 2 *self.t2 *self.t3) * (self.tp - self.ts))/(2* m.pow((self.t2 - self.t3),3))
        self.d2 = -((3 * m.pow(self.t2,2) * self.t3 * self.tp - 6 * self.t2 * m.pow(self.t3, 2) * self.tp + 2 * m.pow(self.t3,3) * self.tp - 2 * m.pow(self.t2,3) * self.ts + 3 * m.pow(self.t2, 2) * self.t3 * self.ts)/(2 * m.pow((self.t2 - self.t3), 3)));

       

    def calculate_rms(self, data):
        """
        Calculate the RMS of the given data step

        Parameters 
        --- 
        data : EMG data 

        Output 
        ---
        root mean square of this data 
        """
        rms = np.sqrt(np.mean(np.square(data)))
        return rms

    def mean_time_series(self, data):
        """
        Calculates the mean time series of the given data step 

        Parameters
        ---
        data : EMG data 

        Output 
        ---
        the mean time series of this data 
        """
        return np.mean(data)

        pass
