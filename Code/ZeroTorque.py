from Controller import *

# P. Pridham, May 2023 

class ZeroTorque(Controller) : 
    """ 
    A class to apply a zero torque controller

    ...

    Attributes
    ----------
    None

    Methods
    -------
    set_parameters(parameters):
        Does nothing, here for compatibility with other controllers
        
    calculate_torque_cmd(state_info):
        Returns a 0 torque command
    """
    def __init__(self) : 
        """
        Constructs all the necessary attributes for the ZeroTorque Controller. 

        Parameters
        ----------
        None
            
        Returns
        -------
        None   
        
        """
        super().__init__()
        
        
    
    def set_parameters(self, **kwargs) :
        """
        Calculates the spline parameters a-d for the curve so we don't need to redo this every loop.  Should be called when the nature of the curve changes.  If all the curve parameters are not set (!=-1) it will not recalculate.

        Parameters
        ----------
        None
             
        Returns
        -------
        None   
        """
        pass
    
    def calculate_torque_cmd(self, **kwargs) : 
        """
        Calculates the current torque command based on the supplied state info, empty. 

        Parameters
        ----------
        None
                
            
        Returns
        -------
        torque_cmd : float  
            0 Nm.
        """
        self.torque_cmd = 0 # store the torque command.
        return self.torque_cmd # return the torque command.

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    # create the ZhangCollins instance
    controller = ZeroTorque()
    # create the parameter dictionary for input.
    
    controller.set_parameters()
    
    percent_gait = np.linspace(0,100,101) # create a set of percent gaits to calculate torque for.
    torque_cmd = [] # create a place to store the torque command output for printing.
    
    for p in percent_gait : # iterate through the different values of percent_gait.
        torque_cmd.append(controller.calculate_torque_cmd()) # append the newly calculated value to the end of the torque_cmd.
        # print(f'ZhangCollins :: __main__ : percent_gait {p} -> torque_cmd {zhang_collins.calculate_torque_cmd([p])}')  # debug statement.
    # print(torque_cmd) # debug statement.
    
    fig, ax = plt.subplots() # create a subplot
    ax.plot(percent_gait, torque_cmd) # plot the torque command

    # add labels to the plot
    ax.set(xlabel='Percent Gait', ylabel='Torque (Nm)',
           title='Torque Profile')
    
    fig.savefig("ZeroTorqueTest.png") # save the plot
    plt.show() # display the plot