import pandas as pd
import numpy as np

class TorqueCalculator: 
    """
    A class used to predict changes in peak torque calculated from GAS, SOL, and TA EMG data 

    Attributes
    ---
    data : EMG data 
    k1 : the scaling constant used for GAS calculation
    k2 : the scaling constant used for SOL calculation
    k3 : the scaling constant used fot TA calculation
    m_ankle: user ankle mass 

    Methods
    ---
    __init__(self, file_name, k1, k2, k3, m_ankle) :
        Initializes the torque calculator based on user inputs 
    
    calculate_torque(num_steps) : 
        Calculates the predicted change in peak torque for the given number of strides of data 

    calculate_rms(data_step) : 
        Calculates the root mean square for one stride of given RMS data 
    
    save_data(output_file_name, torque_changes) : 
        Saved the calculate torque_changes to an output file output_file_name 

    """

    def __init__(self, file_name, k1, k2, k3, m_ankle):
        """
        Initializes a torque calculator system 
        Parameters
        ---
        file_name : the name of the file containing EMG data 
        k1 : GAS scaling constant 
        k2 : SOL scaling constant 
        k3 : TA scaling constant 
        m_ankle : user ankle mass 
        """
        #Load in the EMG data from excel 
        self.data = pd.read_excel(file_name)

        # check the data order (SOL, GAS, TA)
        expected_columns = {'SOL', 'GAS', 'TA'}
        if set(self.data.columns) != expected_columns:
            print("EMG data columns are incorrect.")
            # raise ValueError("EMG data columns are incorrect or in the wrong order.")

        #create NumPy arrays for each muscle 
        self.SOL = self.data['SOL'].to_numpy()
        self.GAS = self.data['GAS'].to_numpy()
        self.TA = self.data['TA'].to_numpy()

        # define the parameters for the torque equation 
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.m_ankle = m_ankle

    def calculate_torque(self, num_steps):
        """
        Calculates the projected change in peak torque for one stride of EMG data 

        Parameters 
        --- 
        num_steps : the number of strides of walking data recorded 

        Output 
        ---
        torque_changes : a vector containing the calculated changes in peak torque for each stride (N)

        """
        points_per_step = len(self.TA) // num_steps
        torque_changes = []

        for i in range(num_steps):
            start_idx = i*points_per_step 
            end_idx = (i+1) * points_per_step

            #Extract one step of data 
            SOL_step = self.SOL[start_idx:end_idx]
            GAS_step = self.GAS[start_idx:end_idx]
            TA_step = self.TA[start_idx:end_idx]

            #perform necessary mathematical calculations on this data 
            SOL_change = self.calculate_rms(SOL_step / np.power(np.mean(SOL_step), self.m_ankle)) 
            GAS_change = self.calculate_rms(GAS_step / np.power(np.mean(GAS_step), self.m_ankle))
            TA_change = self.calculate_rms(TA_step)

            # Calculate the change in peak torque 
            torque_change = (self.k1 * SOL_change + self.k2 * GAS_change - self.k3 * TA_change) / 1000

            # Append the result 
            torque_changes.append(torque_change)

        return torque_changes


    def calculate_rms(self, data_step):
        """
        Calculate the RMS of the given data step

        Parameters 
        --- 
        data_step : EMG data for one stride of walking

        Output 
        ---
        root mean square of this data 
        """
        return np.sqrt(np.mean(np.square(data_step)))


    def save_data(self, output_file_name, torque_changes):
        """
        Save the calculated changes in torque to an output file 

        Parameters
        --- 
        output_file_name : the name of the file to store the calculated changes in peak torque
        torque_changes : a vector containing the calculated change in peak torque for each stride of walking

        """
        np.savetxt(output_file_name, torque_changes, delimiter = ',', header = 'Torque Changes(N)', comments = '')



if __name__ == "__main__":

    # Prompt user input for file name and number of strides 
    file_name = input("Enter the name of the Excel file: ")
    num_steps = int(input("Enter the number of strides of data recorded: "))

    # Prompt user for k1, k2, and k3
    k1 = float(input("Enter the value for SOL scaling (k1) or press enter to use default scaling: ") or 0.1)
    k2 = float(input("Enter the value for GAS scaling (k2) or press enter to use default scaling: ") or 0.1)
    k3 = float(input("Enter the value for TA scaling (k3) or press enter use default scaling: ") or 0.11)

    # Prompt user for ankle mass 
    m_ankle = float(input("Enter the user ankle mass or press enter if unknown: ") or 1.3)

    # Prompt user for the name of the output file 
    output_file_name = input("Enter the name of the output file: ")


    torque_calculator = TorqueCalculator(file_name, k1, k2, k3, m_ankle)
    torque_changes = torque_calculator.calculate_torque(num_steps)
    torque_calculator.save_data(output_file_name, torque_changes)

    print(f"Torque changes have been written to {output_file_name}.")



        