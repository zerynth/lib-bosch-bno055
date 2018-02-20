################################################################################
# Inclination Alarm
#
# Created: 2017-02-24 14:30:21.325536
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
# Eenabled Accelerometer, Magnetometer, and Gyroscope
bno.init("comp") # Operating Mode COMPASS (measure the magnetic earth field and calculate the geographic direction)
print("Ready!")
print("--------------------------------------------------------")

while True:
    data = bno.get_euler() # Read Data
    print('heading: ',data[0],' roll: ',data[1],' pitch: ',data[2])
    angle = int(max(abs(data[1]),abs(data[2]))) # max angle between roll and pitch
    if angle >= 0 and angle<= 30: # angle check
        print("OK! inclination pack: "+str(angle)+" degrees!")
    else:
        print("WARNING! inclination pack: "+ str(angle)+" degrees!")        
    sleep(1000)