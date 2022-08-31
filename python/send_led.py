from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)

ch.setBusOutputControl(canlib.canDRIVER_NORMAL)

ch.busOn()

frame = Frame(id_=0x7FF, data=[5, 100, 10, 00, 10, 20, 10], dlc=7)

ch.write(frame)

frame = Frame(id_=512, data=[0])

ch.write(frame)

time.sleep(0.1)

ch.busOff()