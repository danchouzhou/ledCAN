from canlib import canlib, Frame
import time

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)

ch.setBusOutputControl(canlib.canDRIVER_NORMAL)

ch.busOn()

frame = Frame(id_=0x7FF, data=[0x5A, 0x00, 0x01], dlc=3)

ch.write(frame)

time.sleep(1)

ch.busOff()