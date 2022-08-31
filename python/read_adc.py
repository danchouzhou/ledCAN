from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)
ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
ch.busOn()

# Set the mode to Read ADC
frame = Frame(id_=0x7FF, data=[6], dlc=1)
ch.write(frame)

frame = Frame(id_=0x200, data=0)
ch.write(frame)

while True:
    try:
        frame = ch.read()
        print(frame)
    except canlib.CanNoMsg:
        print
    except KeyboardInterrupt:
        print("Stop.")
        break

ch.busOff()
