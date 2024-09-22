"""
Pseudocode for using a CoadaptiveExo with Cometa and GaitCalculator 
attributes

initialize two boots
initialize one instance of Cometa 
initialize one instance GaitCalculator 

run the boots 

while(no keyboard interrupt)
    append live EMG data to channels 
    append live IMU data to channels 

    when percent_gait = 0.60 (figure out heelstrike time using the GaitCalculator)
        update_parameters for the Coadaptive Exo 
        send torque to the exo with updated parameters 

except KeyBoard interrupt 
"""