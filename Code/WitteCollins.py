from Controller import *
import math as m

# P. Pridham, May 2023 

class WitteCollins(Controller) : 
    """ 
    A class to represent a the Witte/Collins profile from: Witte, K. A., Fiers, P., Sheets-Singer, A. L., & Collins, S. H. (2020). Improving the energy economy of human running with powered and unpowered ankle exoskeleton assistance. Science Robotics, 5(40), eaay9108.

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
        Parameter for the Witte/Collins torque curve rising section : a1 * percent_stance^3 + b1 * self.percent_stance^2 + c1 * percent_stance + d1
    b1 : float
        Parameter for the Witte/Collins torque curve rising section : a1 * percent_stance^3 + b1 * self.percent_stance^2 + c1 * percent_stance + d1
    c1 : float
        Parameter for the Witte/Collins torque curve rising section : a1 * percent_stance^3 + b1 * self.percent_stance^2 + c1 * percent_stance + d1
    d1 : float
        Parameter for the Witte/Collins torque curve rising section : a1 * percent_stance^3 + b1 * self.percent_stance^2 + c1 * percent_stance + d1
        
    a2 : float
        Parameter for the Witte/Collins torque curve falling section : a2 * percent_stance^3 + b2 * self.percent_stance^2 + c2 * percent_stance + d2
    b2 : float
        Parameter for the Witte/Collins torque curve falling section : a2 * percent_stance^3 + b2 * self.percent_stance^2 + c2 * percent_stance + d2
    c2 : float
        Parameter for the Witte/Collins torque curve falling section : a2 * percent_stance^3 + b2 * self.percent_stance^2 + c2 * percent_stance + d2
    d2 : float
        Parameter for the Witte/Collins torque curve falling section : a2 * percent_stance^3 + b2 * self.percent_stance^2 + c2 * percent_stance + d2
        
    percent_stance : float
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
        Constructs all the necessary attributes for the WitteCollins Controller.  Unset parameters are -1.

        Parameters
        ----------
        None
            
        Returns
        -------
        None   
        
        """
        super().__init__()
        
        # Witte/Collins parameters
        self.t0 = -1        # ramp start percent
        self.t1 = -1        # ramp end percent
        self.t2 = -1        # peak percent
        self.t3 = -1        # drop percent
        self.ts = -1        # top torque for the ramp
        self.tp = -1        # peak torque
        self.user_mass = -1    # user mass
        self.peak_torque_normalized = -1    # torque normalized to user mass
        
        # parameters for the Witte/Collins torque curve
        self.a1 = -1
        self.b1 = -1
        self.c1 = -1
        self.d1 = -1
        
        self.a2 = -1
        self.b2 = -1
        self.c2 = -1
        self.d2 = -1
        
        self.percent_stance = -1
    
    
    def set_parameters(self, **kwargs) :
        """
        Calculates the spline parameters a-d for the curve so we don't need to redo this every loop.  Should be called when the nature of the curve changes.  If all the curve parameters are not set (!=-1) it will not recalculate.

        Parameters
        ----------
        Keyword Arguments:
            user_mass : float
                Mass in kg of the person wearing the system.
            ramp_start_percent_stance : float
                The percent gait to start tensioning the belt.
            onset_percent_stance : float
                The percent gait to start ramping up the torque.
            peak_percent_stance : float
                The percent gait the peak torque will occur.
            stop_percent_stance : float
                The percent gait torque will stop being applied.
            onset_torque : float
                Torque Nm the belt will tension to prior to the onset of the push off torque
            normalized_peak_torque : float
                Magnitude of the peak torque normalized to user mass (Nm/kg)
            
        Returns
        -------
        None   
        """
        
        # average values from the witte/collins optimization paper.
        # t0 = 0;
        # t1 = 22.91;
        # t2 = 75.71;
        # t3 = 96.46;
        # ts = 4;

        # peakTorqueNormalized = 0.15; # 0.76; # Using a smaller value due to Dephy Exo Limit.
        
        # Check that the correct parameters came in
        if ("user_mass" in kwargs and kwargs["user_mass"] != -1) :
            self.user_mass = kwargs["user_mass"] # kg
            # print(f'WitteCollins::set_parameters : self.user_mass = {self.user_mass}')
        if ("ramp_start_percent_stance" in kwargs and kwargs["ramp_start_percent_stance"] != -1) :
            self.t0 = kwargs["ramp_start_percent_stance"]
        if ("onset_percent_stance" in kwargs and kwargs["onset_percent_stance"] != -1) :
            self.t1 = kwargs["onset_percent_stance"]
        if ("peak_percent_stance" in kwargs and kwargs["peak_percent_stance"] != -1) :
            self.t2 = kwargs["peak_percent_stance"]
        if ("stop_percent_stance" in kwargs and kwargs["stop_percent_stance"] != -1) :
            self.t3 = kwargs["stop_percent_stance"]
        if ("onset_torque" in kwargs and kwargs["onset_torque"] != -1) :
            self.ts = kwargs["onset_torque"]
        if ("normalized_peak_torque" in kwargs and kwargs["normalized_peak_torque"] != -1) :
            self.peak_torque_normalized = kwargs["normalized_peak_torque"]
        
        # if all the parameters are set calculate the spline parameters
        if (self.user_mass != -1 and self.t0  != -1, self.t1  != -1 and self.t2  != -1 and self.t3  != -1 and self.ts  != -1 and self.peak_torque_normalized  != -1) :
            
            # set limits on how close time points can be to one another
            if ((self.t3-self.t2) < 20) :
                self.t2 = self.t3 - 20
            if ((self.t2-self.t1) < 20) :
                self.t1 = self.t2 - 20
            
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
            print('WitteCollins :: set_parameters : one of the parameters is not set' + \
                '\n user_mass : ' + str(self.user_mass) + \
                '\n ramp_start_percent_stance : ' + str(self.t0) + \
                '\n onset_percent_stance : ' + str(self.t1) + \
                '\n peak_percent_stance : ' + str (self.t2) + \
                '\n stop_percent_stance : ' + str (self.t3) + \
                '\n onset_torque : ' + str (self.ts) + \
                '\n normalized_peak_torque : ' + str (self.peak_torque_normalized))
               
    def calculate_torque_cmd(self, **kwargs) : 
        """
        Calculates the current torque command based on the supplied state info, the percent gait.  If the state info is not set correctly it will output 0.

        Parameters
        ----------
        Keyword Arguments:
            percent_stance : float
                The percent of gait the torque should be calculated for.    
            
        Returns
        -------
        torque_cmd : float  
            The calculated torque command in Nm.
        """
        
        # Check that the state info that came in is correct.
        if ("percent_stance" in kwargs and kwargs["percent_stance"] != -1) :
            self.percent_stance = kwargs["percent_stance"]
        # If something is not set print the needed parameters so the user can see what is not set.
        else :
            print('WitteCollins :: calculate_torque_cmd : some of the data is not set' + \
                '\n percent_stance : ' + str(self.percent_stance))
        tau = 0 # the torque command, set to zero so if we don't make a change it will output 0.
        
        
        if (self.percent_stance != -1) : # check that the percent gait is set
            if ((self.percent_stance <= self.t1)  and  (self.t0 <= self.percent_stance)) : # torque ramp to ts at t1
                tau = self.ts / (self.t1 - self.t0) * self.percent_stance - self.ts/(self.t1 - self.t0) * self.t0;
            
            elif (self.percent_stance <= self.t2) : # the rising spline
                tau = self.a1 * m.pow(self.percent_stance,3) + self.b1 * m.pow(self.percent_stance,2) + self.c1 * self.percent_stance + self.d1;
                
            elif (self.percent_stance <= self.t3) : # the falling spline
               tau = self.a2 * m.pow(self.percent_stance,3) + self.b2 * m.pow(self.percent_stance,2) + self.c2 * self.percent_stance + self.d2;
                
            else : # go to the slack position if we aren't commanding a specific value
                tau = 0;
                
        
        self.torque_cmd = tau # store the torque command.
        return tau # return the torque command.

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    # create the WitteCollins instance
    witte_collins = WitteCollins()
    
    # t0 = 0;
    # t1 = 22.91;
    # t2 = 75.71;
    # t3 = 96.46;
    witte_collins.set_parameters(user_mass = 100, ramp_start_percent_stance = 0, onset_percent_stance = 22.91, peak_percent_stance = 75.71, stop_percent_stance = 96.46, onset_torque = 4, normalized_peak_torque = .2)
    
    
    percent_stance = np.linspace(0,100,101) # create a set of percent gaits to calculate torque for.
    torque_cmd = [] # create a place to store the torque command output for printing.
        
    for p in percent_stance : # iterate through the different values of percent_stance.
        torque_cmd.append(witte_collins.calculate_torque_cmd(percent_stance = p)) # append the newly calculated value to the end of the torque_cmd.
        # print(f'WitteCollins :: __main__ : percent_stance {p} -> torque_cmd {witte_collins.calculate_torque_cmd([p])}')  # debug statement.
    # print(torque_cmd) # debug statement.
    
    fig, ax = plt.subplots() # create a subplot
    ax.plot(percent_stance, torque_cmd) # plot the torque command

    # add labels to the plot
    ax.set(xlabel='Percent Gait', ylabel='Torque (Nm)',
           title='Witte Collins Torque Profile')
    
    fig.savefig("WitteCollinsTest.png") # save the plot
    plt.show() # display the plot