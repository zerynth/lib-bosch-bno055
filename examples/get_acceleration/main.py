################################################################################
# Get Acceleration Example
#
# Created: 2017-02-24 12:48:43.342375
#
################################################################################

from bosch.bno055 import bno055
import streams

streams.serial()

# Setup sensor 
# This setup is referred to bno055 mounted on 10DOF Click in slot A of a Flip n Click device 

print("Start...")
bno = bno055.BNO055(I2C0)
bno.start()
print("Init...")
# Enabled Accelerometer
bno.init("acc") # Operating Mode ACCONLY (only raw accelerometer data)
print("Ready!")
print("--------------------------------------------------------")

while True:
    data = bno.get_acc("x") # Read Data on x axis
    print("Acceleration on X axis", data)
    data = bno.get_acc("y") # Read Data on y axis
    print("Acceleration on Y axis", data)
    data = bno.get_acc("Z") # Read Data on z axis
    print("Acceleration on Z axis", data)
    data = bno.get_acc() # Read Data on 3 axis
    print("Acceleration on XYZ", data)
    print("--------------------------------------------------------")
    sleep(5000)