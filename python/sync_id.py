from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

# Set the synchronize ID = 0x100
frame = Frame(id_=0x7FF, data=[0x5A, 0x00, 0x01], dlc=3)
ch.write(frame)

# Set the mode to fill, number of pixel=100, Red=60, Green=30, Blue=50
frame = Frame(id_=0x7FF, data=[1, 100, 60, 30, 50], dlc=5)
ch.write(frame)

# light up LED strip
frame = Frame(id_=0x100, data=0)
ch.write(frame)

# Ensure that the frame has been send then turn off the CAN adapter
time.sleep(1)
ch.busOff()
