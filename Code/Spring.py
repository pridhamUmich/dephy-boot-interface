from Controller import *
import math as m

# P. Pridham, May 2023 

class Spring(Controller) : 
    """ 
    A class to apply a spring with a linear profile.

    ...

    Attributes
    ----------
    zero_angle : float
        The value of the neutral angle position (rad)
    current_angle : float
        Current joint angle (rad)
    ks : float
        Spring constant Nm/rad

    Methods
    -------
    set_parameters(parameters):
        Sets the neutral position and spring constant
        
    calculate_torque_cmd(state_info):
        Calculates the current torque command based on the the ankle angle.
    """
    def __init__(self) : 
        """
        Constructs all the necessary attributes for the Spring Controller.  Unset parameters are -1.

        Parameters
        ----------
        None
            
        Returns
        -------
        None   
        
        """
        super().__init__()
        
        # parameters
        self.zero_angle = -1    # ramp start percent
        self.current_angle = -1 # ramp end percent
        self.ks = -1            # Spring constant Nm/rad
    
    
    def set_parameters(self, **kwargs) :
        """
        Sets the zero angle and spring constant for the controller.

        Parameters
        ----------
        Keyword Arguments:
            zero_angle : float
                Neutral angle (rad) for the joint
            ks : float
                Spring constant Nm/rad
            
        Returns
        -------
        None   
        """
        
        # Check that the correct parameters came in
        if ("zero_angle" in kwargs and kwargs["zero_angle"] != -1) :
            self.zero_angle = kwargs["zero_angle"] # rad
        if ("ks" in kwargs and kwargs["ks"] != -1) :
            self.ks = kwargs["ks"]
        
        # if all the parameters are set we are good
        if (self.zero_angle != -1 and self.ks  != -1) :
            pass
            
        # if not everything is set print the inputs so the user can see what is not set.    
        else :
            print('Spring :: set_parameters : one of the parameters is not set' + \
                '\n zero_angle : ' + str(self.zero_angle) + \
                '\n ks : ' + str(self.ks))
               
    def calculate_torque_cmd(self, **kwargs) : 
        """
        Calculates the current torque command based on the supplied state info, current angle.  If the state info is not set correctly it will output 0.

        Parameters
        ----------
        Keyword Arguments:
            current_angle : float
                The current angle of the joint (rad)    
            
        Returns
        -------
        torque_cmd : float  
            The calculated torque command in Nm.
        """
        
        # Check that the state info that came in is correct.
        if ("current_angle" in kwargs and kwargs["current_angle"] != -1) :
            self.current_angle = kwargs["current_angle"]
        # If something is not set print the needed parameters so the user can see what is not set.
        else :
            print('Spring :: calculate_torque_cmd : some of the data is not set' + \
                '\n current_angle : ' + str(self.current_angle))
        tau = 0 # the torque command, set to zero so if we don't make a change it will output 0.
        
        
        if (self.current_angle != -1) : # check that the percent gait is set
            if (self.current_angle <= 0) : # saturate the torque
                tau = self.zero_angle * self.ks
            
            elif ((self.current_angle >= self.zero_angle)) : # stay zero if past the zero angle
                tau = 0
                
            else : # linear calculation of spring torque
                tau = (self.zero_angle - self.current_angle) * self.ks;
                
        
        self.torque_cmd = tau # store the torque command.
        return tau # return the torque command.

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    # create the ZhangCollins instance
    spring_controller = Spring()
    
    spring_controller.set_parameters(zero_angle = 50 * m.pi/180, ks = 10)
    
    
    angles = np.linspace(-m.pi/2,m.pi/2,101) # create a set of angles to calculate torque for.
    torque_cmd = [] # create a place to store the torque command output for printing.
        
    for p in angles : # iterate through the different values of percent_gait.
        torque_cmd.append(spring_controller.calculate_torque_cmd(current_angle = p)) # append the newly calculated value to the end of the torque_cmd.
    
    fig, ax = plt.subplots() # create a subplot
    ax.plot(angles, torque_cmd) # plot the torque command

    # add labels to the plot
    ax.set(xlabel='Angle', ylabel='Torque (Nm)',
           title='Spring Torque Profile')
    
    fig.savefig("springTest.png") # save the plot
    plt.show() # display the plot