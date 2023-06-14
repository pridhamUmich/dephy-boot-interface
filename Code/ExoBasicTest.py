from flexsea.device import Device

boot_firmware_version = "7.2.0"

# left_port = "COM1"
right_port = "COM4"

streaming_frequency = 500

# set data values to pull
data_of_interest = [
"statetime",
"gyro_x",
"gyro_y",
"gyro_z",
"accl_x",
"accl_y",
"accl_z",
"motor_voltage",
"motor_current",
"motor_ang",
"motor_vel",
"ank_ang",
"ank_vel",
]

# set PID values to send
pid_val_current_control = {
    "kp": 40,
    "ki": 400,
    "kd": 0,
    "k": 0,
    "b": 0,
    "ff": 0,
}

# left_boot = Device(port=left_port, cLibVersion=boot_firmware_version)
right_boot = Device(port=right_port, firmwareVersion=boot_firmware_version)

# left_boot.open()
right_boot.open()

# left_boot.start_streaming(streaming_frequency)
right_boot.start_streaming(streaming_frequency)

right_boot.set_gains(**gains)

run_loop = true;

try :
    while run_loop :
        # left_data = left_boot.read(allData=true)
        right_data = right_boot.read(allData=false)

        # left_boot.print()
        right_boot.print(right_data)
        
        current = 500; # mA
        # left_boot.command_motor_current(current) # milliamps
        right_boot.command_motor_current(current) # milliamps
except KeyboardInterrupt:
			print("KeyboardInterrupt has been caught.")     

# left_boot.close()
right_boot.close() # this also stops streaming.

