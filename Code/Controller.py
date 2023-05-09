""" @package Controller
abstract class for controllers
Should have an:
__init__()
set_parameters(parameter_array)
calulate_cmd(state_array)


"""
import abc #abstract base class

class Controller(abc.ABC) :
    @abc.abstractmethod
    def __init__(self) :
        self.torque_cmd = 0
    
    @abc.abstractmethod
    def set_parameters(self, parameter_array) :
        pass
    
    @abc.abstractmethod
    def calculate_torque_cmd(self, state_array) :
        pass
    
