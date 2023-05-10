from Controller import *
import math as m

class ZhangCollins(Controller) : 
    """ 
    A class to represent a the Zhang/Collins profile from: Zhang, J., Fiers, P., Witte, K. A., Jackson, R. W., Poggensee, K. L., Atkeson, C. G., & Collins, S. H. (2017). Human-in-the-loop optimization of exoskeleton assistance during walking. Science, 356(6344), 1280-1284.

    ...

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
    
    
    def set_parameters(self, parameters) :
        """
        Calculates the spline parameters a-d for the curve so we don't need to redo this every loop.  Should be called when the nature of the curve changes.  If all the curve parameters are not set (!=-1) it will not recalculate.

        Parameters
        ----------
        parameters : dict
            Parameters for the Zhang/Collins torque profile, keys :
                "user_mass"
                "ramp_start_percent_gait"
                "onset_percent_gait"
                "peak_percent_gait"
                "stop_percent_gait"
                "onset_torque"
                "normalized_peak_torque"
            
            
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
        if ("user_mass" in parameters and parameters["user_mass"] != -1) :
            self.user_mass = parameters["user_mass"] # kg
            # print(f'ZhangCollins::set_parameters : self.user_mass = {self.user_mass}')
        if ("ramp_start_percent_gait" in parameters and parameters["ramp_start_percent_gait"] != -1) :
            self.t0 = parameters["ramp_start_percent_gait"]
        if ("onset_percent_gait" in parameters and parameters["onset_percent_gait"] != -1) :
            self.t1 = parameters["onset_percent_gait"]
        if ("peak_percent_gait" in parameters and parameters["peak_percent_gait"] != -1) :
            self.t2 = parameters["peak_percent_gait"]
        if ("stop_percent_gait" in parameters and parameters["stop_percent_gait"] != -1) :
            self.t3 = parameters["stop_percent_gait"]
        if ("onset_torque" in parameters and parameters["onset_torque"] != -1) :
            self.ts = parameters["onset_torque"]
        if ("normalized_peak_torque" in parameters and parameters["normalized_peak_torque"] != -1) :
            self.peak_torque_normalized = parameters["normalized_peak_torque"]
          
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
            
        else :
            print('ZhangCollins :: set_parameters : one of the parameters is not set' + \
                '\n user_mass : ' + str(self.user_mass) + \
                '\n ramp_start_percent_gait : ' + str(self.t0) + \
                '\n onset_percent_gait : ' + str(self.t1) + \
                '\n peak_percent_gait : ' + str (self.t2) + \
                '\n stop_percent_gait : ' + str (self.t3) + \
                '\n onset_torque : ' + str (self.ts) + \
                '\n normalized_peak_torque : ' + str (self.peak_torque_normalized))
    # @classmethod            
    def calculate_torque_cmd(self, state_info) : 
        """
        Calculates the current torque command based on the supplied state info, the percent gait.  If the state info is not set correctly it will output 0.

        Parameters
        ----------
        state_info : dict
            State info from the exo and person used to calculate the Zhang/Collins torque profile, keys :
                "percent_gait"
                
            
        Returns
        -------
        torque_cmd : float  
            The calculated torque command in Nm.
        """
        # update data
        
        if ("percent_gait" in state_info and state_info["percent_gait"] != -1) :
            self.percent_gait = state_info["percent_gait"]
        else :
            print('ZhangCollins :: calculate_torque_cmd : some of the data not set' + \
                '\n percent_gait : ' + str(self.percent_gait))
        tau = 0
                
        if (self.percent_gait != -1) : 
            if ((self.percent_gait <= self.t1)  and  (self.t0 <= self.percent_gait)) : # torque ramp to ts at t1
                tau = self.ts / (self.t1 - self.t0) * self.percent_gait - self.ts/(self.t1 - self.t0) * self.t0;
            
            elif (self.percent_gait <= self.t2) : # the rising spline
                tau = self.a1 * m.pow(self.percent_gait,3) + self.b1 * m.pow(self.percent_gait,2) + self.c1 * self.percent_gait + self.d1;
                
            elif (self.percent_gait <= self.t3) : # the falling spline
               tau = self.a2 * m.pow(self.percent_gait,3) + self.b2 * m.pow(self.percent_gait,2) + self.c2 * self.percent_gait + self.d2;
                
            else : # go to the slack position if we aren't commanding a specific value
                tau = 0;
                
        
        self.torque_cmd = tau
        return tau

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    zhang_collins = ZhangCollins()
    parameters = {
        "user_mass" : 100,
        "ramp_start_percent_gait" : 0,
        "onset_percent_gait" : 27.1,
        "peak_percent_gait" : 50.4,
        "stop_percent_gait" : 62.7,
        "onset_torque" : 2,
        "normalized_peak_torque" : .2,
    }
    
    zhang_collins.set_parameters(parameters) #[100, 0, 27.1, 50.4, 62.7, 2, .2])
    
    percent_gait = np.linspace(0,100,101)
    torque_cmd = []
    state_info = {
        "percent_gait" : -1
    }
    
    for p in percent_gait :
        state_info["percent_gait"] = p
        torque_cmd.append(zhang_collins.calculate_torque_cmd(state_info))
        # print(f'ZhangCollins :: __main__ : percent_gait {p} -> torque_cmd {zhang_collins.calculate_torque_cmd([p])}')
    # print(torque_cmd)
    fig, ax = plt.subplots()
    ax.plot(percent_gait, torque_cmd)

    ax.set(xlabel='Percent Gait', ylabel='Torque (Nm)',
           title='Zhang Collins Torque Profile')
    
    fig.savefig("ZhangCollinsTest.png")
    plt.show()