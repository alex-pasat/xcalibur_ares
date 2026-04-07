import serial
import time
from enum import Enum
import numpy as np
from datetime import datetime

class State(Enum):
    CONNECT      = 0
    WAIT_START   = 1
    DETECT_BLADE = 2
    SEND_BLADE_DETECTED = 3
    WAIT_VISION  = 4
    VISION = 5

START = 'START'
START_VISION = 'START_VISION'
RECONNECT = "RECONNECT"
KNIFE_DETECTED = "KNIFE_DETECTED"
FAIL = "FAIL"

def open_port():
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
            log("Connected")
            return ser
        except serial.SerialException:
            log("Waiting for device...")
            time.sleep(1)

def log(msg):
    print(msg)
    with open("received.txt", "a") as f:
        f.write(msg + "\n")

def detect_blade():
    time.sleep(1)
    return True

def send_array(ser):
    data = np.load("/home/chuddycholo/XCalibur-Vision-IK/knife_data.npz")

    tip_q1 = data['tip_q1']
    tip_q2 = data['tip_q2']
    yaw_indices = data['yaw_indices']
    ratios1 = data['ratios1']
    ratios2 = data['ratios2']

    N = np.int16(len(yaw_indices))

    log(f"N {N}")
    log(f"tip_q1 {tip_q1}")
    log(f"tip_q2 {tip_q2}")
    log(f"yaw_indices {yaw_indices}")
    log(f"ratios1 {ratios1}")

    block_1 = bytes()
    block_1 += (tip_q1.astype(np.float64).tobytes() + tip_q2.astype(np.float64).tobytes()[:3])

    block_2 = bytes()
    block_2 += (tip_q2.astype(np.float64).tobytes()[3:]+ N.astype(np.int16).tobytes())

    CHUNK_SIZE = 64

    ser.write(block_1)
    time.sleep(0.02)
    log("Sent block 1")

    ser.write(block_2)
    time.sleep(0.02)
    log("Sent block 2")

    time.sleep(1)

    for i in range(0, N, 8):
        block = bytes()
        block += (yaw_indices[i:i+8]).astype(np.float64).tobytes()
        ser.write(block)
        time.sleep(0.02)
        log(str(yaw_indices[i:i+8]))

    log("sent yaw")
    time.sleep(1)

    for i in range(0, N+1, 2):
        block = bytes()
        block += (ratios1[i:i+2]).astype(np.float64).tobytes()
        ser.write(block)
        time.sleep(0.02)

    log("sent ratios1")

    for i in range(0, N+1, 2):
        block = bytes()
        block += (ratios2[i:i+2]).astype(np.float64).tobytes()
        ser.write(block)
        time.sleep(0.02)

    log("sent ratios2")
    log("Done sending")

    return True

def main():
    state = State.CONNECT
    ser = None

    with open("received.txt", "a") as f:
        f.write(f"\n--- Session started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")

    while True:
        if state == State.CONNECT:
            ser = open_port()
            try:
                ser.write(RECONNECT.encode('utf-8'))
                state = State.WAIT_START
            except serial.SerialException:
                log("Lost connection")
                ser.close()
                state = State.CONNECT

        elif state == State.WAIT_START:
            try:
                line = ser.readline().decode().strip()
                if START in line:
                    log("Got START")
                    state = State.DETECT_BLADE
            except serial.SerialException:
                log("Lost connection")
                ser.close()
                state = State.CONNECT

        elif state == State.DETECT_BLADE:
            result = detect_blade()
            if not result:
                try:
                    ser.write(FAIL.encode('utf-8'))
                    log("Knife Detection Failed!")
                    state = State.WAIT_START
                except serial.SerialException:
                    log("Lost connection")
                    ser.close()
                    state = State.CONNECT
                continue
            try:
                ser.write(KNIFE_DETECTED.encode('utf-8'))
                log("Wrote knife detected!")
                state = State.WAIT_VISION
            except serial.SerialException:
                log("Lost connection")
                ser.close()
                state = State.CONNECT

        elif state == State.WAIT_VISION:
            log("Lost connection")
            ser.close()
            state = State.CONNECT

        elif state == State.VISION:
            result = send_array(ser)
            if not result:
                try:
                    ser.write(FAIL.encode('utf-8'))
                    log("Profile Detection Failed!")
                    state = State.WAIT_START
                except serial.SerialException:
                    log("Lost connection")
                    ser.close()
                    state = State.CONNECT
                continue
            state = State.WAIT_START


if __name__ == "__main__":
    main()
