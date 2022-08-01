from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=1, bitrate=canlib.Bitrate.BITRATE_500K)

ch.setBusOutputControl(canlib.canDRIVER_NORMAL)

ch.busOn()

frame = Frame(id_=19, data=[8, 0, 0], dlc=3)

ch.write(frame)

time.sleep(0.01)

frame = Frame(id_=512, data=[0])

ch.write(frame)

time.sleep(0.01)

ch.busOff()