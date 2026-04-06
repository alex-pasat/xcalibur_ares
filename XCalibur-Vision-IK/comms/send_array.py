import serial
import time
import numpy as np

# Load the data
data = np.load("/home/chuddycholo/XCalibur-Vision-IK/knife_data.npz")

# Open the port
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    timeout=1
)

time.sleep(2)

tip_q1 = data['tip_q1']
tip_q2 = data['tip_q2']
yaw_indices = data['yaw_indices']
ratios1 = data['ratios1']
ratios2 = data['ratios2']

N = np.int16(len(yaw_indices))

print("N", N)
print("tip_q1",tip_q1)
print("tip_q2",tip_q2)
print("yaw_indices",yaw_indices)
print("ratios1",ratios1)

# put size and tip_q1 in a block
block_1 = bytes()
block_1 += (tip_q1.astype(np.float64).tobytes() + tip_q2.astype(np.float64).tobytes()[:3])

block_2 = bytes()
block_2 += (tip_q2.astype(np.float64).tobytes()[3:]+ N.astype(np.int16).tobytes())

# buffer = bytes()
# for arr in (yaw_indices, ratios1, ratios2):
#     buffer += arr.astype(np.float64).tobytes()  # convert to float32 raw bytes

# print(f"Total bytes to send: {len(buffer)}")

# Send in chunks (serial buffer is usually 64–256 bytes)
CHUNK_SIZE = 64

ser.write(block_1)
time.sleep(0.02)  # small delay between chunks
print(f"Sent block 1")

ser.write(block_2)
time.sleep(0.02)  # small delay between chunks
print(f"Sent block 2")

time.sleep(1)

'''
block = bytes()
block += (yaw_indices[16:24]).astype(np.float64).tobytes()
ser.write(block)
time.sleep(0.02)
'''

for i in range(0, N, 8):
    block = bytes()
    block += (yaw_indices[i:i+8]).astype(np.float64).tobytes()
    ser.write(block)
    time.sleep(0.02)  # small delay between chunks
    print(yaw_indices[i:i+8])

print("sent yaw")
time.sleep(1)

for i in range(0, N+1, 2):
    block = bytes()
    block += (ratios1[i:i+2]).astype(np.float64).tobytes()
    #print(block)
    ser.write(block)
    time.sleep(0.02)  # small delay between chunks
    #print(f"Sent bytes {i}")

print("sent ratios1")

for i in range(0, N+1, 2):
    block = bytes()
    block += (ratios2[i:i+2]).astype(np.float64).tobytes()
    ser.write(block)
    time.sleep(0.02)  # small delay between chunks
    # print(f"Sent bytes {i} to {i+len(block)}")

print("sent ratios2")
'''
print("N", N)
print("tip_q1",tip_q1)
print("tip_q2",tip_q2)
print("yaw_indices",yaw_indices)
print("ratios1",ratios1)
'''
# for i in range(0, N//CHUNK_SIZE+1, CHUNK_SIZE):
#     chunk = buffer[i:i+CHUNK_SIZE]
#     ser.write(chunk)
#     time.sleep(0.02)  # small delay between chunks
#     # print(f"Sent bytes {i} to {i+len(chunk)}")
# time.sleep(0.1)
# response = ser.read(64)
# if response:
#     print(f"Received: {list(response)}")
# else:
#     print("No response")

# for i in range(N//CHUNK_SIZE+1, N//CHUNK_SIZE+1 + (4*(N-1))//CHUNK_SIZE+1, CHUNK_SIZE):
#     chunk = buffer[i:i+CHUNK_SIZE]
#     ser.write(chunk)
#     time.sleep(0.02)  # small delay between chunks
#     # print(f"Sent bytes {i} to {i+len(chunk)}")
# time.sleep(0.1)
# response = ser.read(64)
# if response:
#     print(f"Received: {list(response)}")
# else:
#     print("No response")

# for i in range(N//CHUNK_SIZE+1 + (4*(N-1))//CHUNK_SIZE+1, len(buffer), CHUNK_SIZE):
#     chunk = buffer[i:i+CHUNK_SIZE]
#     ser.write(chunk)
#     time.sleep(0.02)  # small delay between chunks
#     # print(f"Sent bytes {i} to {i+len(chunk)}")
# time.sleep(0.1)
# response = ser.read(64)
# if response:
#     print(f"Received: {list(response)}")
# else:
#     print("No response")

print("Done sending")

# Wait for response
response = ser.read(64)
if response:
    print(f"Received: {list(response)}")
else:
    print("No response")

ser.close()
