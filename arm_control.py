from serial import Serial
import time

port = "/dev/tty.usbserial-1120"
baud = 115200
joint_min = 0.0
joint_max = 180.0
gripper_max = 90.0

ser = Serial(port, baud, timeout=1)
time.sleep(2)

def send_angles(base=90.0, shoulder=90.0, elbow=90.0, wrist_roll=90.0, wrist_pitch=90.0, gripper=0.0, wait=0.5):
    cmd = f"{base},{shoulder},{elbow},{wrist_roll},{wrist_pitch},{gripper}\n"
    ser.write(cmd.encode())
    print("Sent:", cmd.strip())
    time.sleep(wait)

angles = [90, 90, 90, 90, 20, 45]
send_angles(*angles)

while ser.in_waiting:
    print(ser.readline().decode().strip())
