from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

# Set the mode to breath, number of pixel=100, Red=20, Green=100, Blue=20, period 10 seconds
frame = Frame(id_=0x7FF, data=[4, 100, 20, 100, 20, 10], dlc=6)
ch.write(frame)

while True:
    try:
        frame = Frame(id_=0x200, data=0)
        ch.write(frame)

        # Extend the time for a while, ensure the progress finished
        time.sleep(11)
    except KeyboardInterrupt:
        print("Stop.")
        break

ch.busOff()
