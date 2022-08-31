from canlib import canlib, Frame
import time
import shutil

ch = canlib.openChannel(channel=0, bitrate=canlib.Bitrate.BITRATE_500K)

ch.setBusOutputControl(canlib.canDRIVER_NORMAL)

ch.busOn()

frame = Frame(id_=0x7FF, data=[6, 100, 10, 00, 10, 20, 10], dlc=7)

ch.write(frame)

time.sleep(0.01)

frame = Frame(id_=512, data=[0])

ch.write(frame)

time.sleep(0.01)

width, height = shutil.get_terminal_size((80, 20))

def printframe(frame, width):
    form = '═^' + str(width - 1)
    print(format(" Frame received ", form))
    print("id:", frame.id)
    print("data:", bytes(frame.data))
    print("dlc:", frame.dlc)
    print("flags:", frame.flags)
    print("timestamp:", frame.timestamp)

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