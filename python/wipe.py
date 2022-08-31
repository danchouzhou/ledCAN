from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

while True:
    try:
        # Set the mode to wipe, number of pixel=100, Red=10, Green=60, Blue=50, increase every 10ms
        frame = Frame(id_=0x7FF, data=[2, 100, 10, 60, 50, 10], dlc=6)
        ch.write(frame)

        frame = Frame(id_=0x200, data=0)
        ch.write(frame)

        time.sleep(2.5)

        # Set the mode to wipe, number of pixel=100, Red=0, Green=0, Blue=0, increase every 10ms
        frame = Frame(id_=0x7FF, data=[2, 100, 0, 0, 0, 10], dlc=6)
        ch.write(frame)

        frame = Frame(id_=0x200, data=0)
        ch.write(frame)

        time.sleep(2.5)
    except KeyboardInterrupt:
        print("Stop.")
        break

ch.busOff()
