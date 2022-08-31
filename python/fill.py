from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

# Set the mode to fill, number of pixel=100, Red=100, Green=7, Blue=0
frame = Frame(id_=0x7FF, data=[1, 100, 100, 7, 0], dlc=5)
ch.write(frame)

# light up LED strip
frame = Frame(id_=0x200, data=0)
ch.write(frame)

# Ensure that the frame has been send then turn off the CAN adapter
time.sleep(1)
ch.busOff()
