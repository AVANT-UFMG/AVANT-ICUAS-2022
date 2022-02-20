import time

from ..DroneCMD import DroneCMD

if __name__ == "__main__":
    drone_cmd = DroneCMD()
    drone_cmd.takeoff(height=6.0)
    time.sleep(1)
    drone_cmd.land()