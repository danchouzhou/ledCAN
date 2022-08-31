from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

# Set the mode to scroll, number of pixel=100, Red=50, Green=0, Blue=50, lenght of snake 20 pixels, move every 10 ms
frame = Frame(id_=0x7FF, data=[5, 100, 50, 0, 50, 20, 10], dlc=7)
ch.write(frame)

while True:
    try:
        frame = Frame(id_=0x200, data=0)
        ch.write(frame)

        # Extend the time for a while, ensure the progress finished
        time.sleep(5)
    except KeyboardInterrupt:
        print("Stop.")
        break

ch.busOff()
