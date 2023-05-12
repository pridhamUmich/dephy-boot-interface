import abc #abstract base class

# P. Pridham, May 2023 

class Controller(abc.ABC) :
    """ 
    An abstract class to define the calls to a Controller which outputs a torque command.

    ...

    Attributes
    ----------
    torque_cmd : float
        The most recent calculated torque command

    Methods
    -------
    set_parameters(parameters):
        Sets the parameters for the controller and calculates any new values that may be needed when parameters change.
        
    calculate_torque_cmd(state_info):
        Calculates the current torque command based on the supplied state info.  If the state info is not set correctly it will output 0.
    """
    
    @abc.abstractmethod
    def __init__(self) :
        """
        Constructs all the necessary attributes for the Controller.  Should be called in the inheriting class's __init__.

        Parameters
        ----------
        None
            
        Returns
        -------
        None   
        
        """
        self.torque_cmd = 0
    
    @abc.abstractmethod
    def set_parameters(self, **kwargs) :
        """
        Sets the parameters for the controller and calculates any new values that may be needed when parameters change.

        Parameters
        ----------
        parameters : dict
            Parameters for the torque calculation, keys should be defined in the inheriting class during override.
            
        Returns
        -------
        None   
        """
        pass
    
    @abc.abstractmethod
    def calculate_torque_cmd(self, **kwargs) :
        """
        Calculates the current torque command based on the supplied state info.  If the state info is not set correctly it will output 0.

        Parameters
        ----------
        state_info : dict
            State info from the exo and person used to calculate the torque command, keys should be defined in the inheriting class during override.
            
        Returns
        -------
        torque_cmd : float  
            The calculated torque command in Nm.
        """
        pass
    
