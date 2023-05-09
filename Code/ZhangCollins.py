from Controller import *
import math as m

class ZhangCollins(Controller) : 
    
    def __init__(self) : 
        super().__init__()
        
        # self.torque_cmd = 0
        
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
        
        # indexes for the parameter array.
        self.mass_idx = 0
        self.ramp_start_percent_gait_idx = 1
        self.onset_percent_gait_idx = 2
        self.peak_percent_gait_idx = 3
        self.stop_percent_gait_idx = 4
        self.onset_torque_idx = 5
        self.normalized_peak_torque_idx = 6
        
        # state array indexes
        self.percent_gait_idx = 0;
        
        self.percent_gait = -1
    
    # @classmethod
    def set_parameters(self, parameter_array) :
        # average values from the zhang/collins optimization paper.
        # t0 = 0;
        # t1 = 27.1;
        # t2 = 50.4;
        # t3 = 62.7;
        # ts = 2;

        # peakTorqueNormalized = 0.20; # 0.76; # Using a smaller value due to Dephy Exo Limit.
        
        # print(f'ZhangCollins::set_parameters : parameter_array[self.mass_idx] = {parameter_array[self.mass_idx]}')
        if (parameter_array[self.mass_idx] != -1) :
            self.user_mass = parameter_array[self.mass_idx] # kg
            # print(f'ZhangCollins::set_parameters : self.user_mass = {self.user_mass}')
        if (parameter_array[self.ramp_start_percent_gait_idx] != -1) :
            self.t0 = parameter_array[self.ramp_start_percent_gait_idx]
        if (parameter_array[self.onset_percent_gait_idx] != -1) :
            self.t1 = parameter_array[self.onset_percent_gait_idx]
        if (parameter_array[self.peak_percent_gait_idx] != -1) :
            self.t2 = parameter_array[self.peak_percent_gait_idx]
        if (parameter_array[self.stop_percent_gait_idx] != -1) :
            self.t3 = parameter_array[self.stop_percent_gait_idx]
        if (parameter_array[self.onset_torque_idx] != -1) :
            self.ts = parameter_array[self.onset_torque_idx]
        if (parameter_array[self.normalized_peak_torque_idx] != -1) :
            self.peak_torque_normalized = parameter_array[self.normalized_peak_torque_idx]
        
            
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
            print('ExoBoot :: init_collins_profile : one of the parameters is not set' + \
                '\n user_mass : ' + str(self.user_mass) + \
                '\n ramp_start_percent_gait : ' + str(self.t0) + \
                '\n onset_percent_gait : ' + str(self.t1) + \
                '\n peak_percent_gait : ' + str (self.t2) + \
                '\n stop_percent_gait : ' + str (self.t3) + \
                '\n onset_torque : ' + str (self.ts) + \
                '\n normalized_peak_torque : ' + str (self.peak_torque_normalized))
    # @classmethod            
    def calculate_torque_cmd(self, state_array) : 
        # update data
        self.percent_gait = state_array[self.percent_gait_idx]
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
    zhang_collins.set_parameters([100, 0, 27.1, 50.4, 62.7, 2, .2])
    
    percent_gait = np.linspace(0,100,101)
    torque_cmd = []
    
    for p in percent_gait :
        torque_cmd.append(zhang_collins.calculate_torque_cmd([p]))
        # print(f'ZhangCollins :: __main__ : percent_gait {p} -> torque_cmd {zhang_collins.calculate_torque_cmd([p])}')
    # print(torque_cmd)
    fig, ax = plt.subplots()
    ax.plot(percent_gait, torque_cmd)

    ax.set(xlabel='Percent Gait', ylabel='Torque (Nm)',
           title='Zhang Collins Torque Profile')
    
    fig.savefig("ZhangCollinsTest.png")
    plt.show()