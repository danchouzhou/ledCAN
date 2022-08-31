from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

# Set the mode to blink, number of pixel=100, Red=100, Green=35, Blue=2, light up 500 ms
frame = Frame(id_=0x7FF, data=[3, 100, 100, 35, 2, 50], dlc=6)
ch.write(frame)

while True:
    try:
        # light up LED strip every one sencod, will automatically turn off in 500ms
        frame = Frame(id_=0x200, data=0)
        ch.write(frame)

        time.sleep(1)
    except KeyboardInterrupt:
        print("Stop.")
        break

ch.busOff()
