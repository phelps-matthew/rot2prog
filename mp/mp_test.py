import rot2prog
import logging
import time


logging.basicConfig(level = logging.DEBUG)

if __name__ == "__main__":
    rot = rot2prog.ROT2Prog('/dev/ttyUSB0')
    rot.set(0, 0)
    rot.status()
    #rot.set(12.63, 10.49)