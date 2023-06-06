from flexsea.device import Device
# import sys
# sys.path.insert(0,'C:\\Users\\pridham\\.dephy\\precompiled_c_libs\\7.2.0\\windows_64bit')


boot_firmware_version = "7.2.0"

# left_port = "COM1"
right_port = "COM4"

streaming_frequency = 500

# left_boot = Device(port=left_port, cLibVersion=boot_firmware_version)
right_boot = Device(port=right_port, firmwareVersion=boot_firmware_version)

# left_boot.open()
right_boot.open()

# left_boot.start_streaming(streaming_frequency)
right_boot.start_streaming(streaming_frequency)

run_loop = true;

try :
    while run_loop :
        # left_data = left_boot.read(allData=true)
        right_data = right_boot.read(allData=true)

        # left_boot.print()
        right_boot.print()
        
        current = 500; # mA
        # left_boot.command_motor_current(current) # milliamps
        right_boot.command_motor_current(current) # milliamps
except KeyboardInterrupt:
			print("KeyboardInterrupt has been caught.")     

# left_boot.close()
right_boot.close() # this also stops streaming.

