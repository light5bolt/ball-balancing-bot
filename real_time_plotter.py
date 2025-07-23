import serial.tools.list_ports
import serial
import time
import matplotlib.pyplot as plt
from collections import deque

def real_time_plotter(port, baudrate=115200):

    # Setup serial connection
    ser = serial.Serial(port, baudrate, timeout=1)
    ser.setDTR(False)
    time.sleep(1)
    ser.flushInput()
    ser.setDTR(True)
    time.sleep(1)

    # Setup plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 8))

    # Data storage
    ball_x_data = deque(maxlen=200)
    ball_y_data = deque(maxlen=200)
    target_x_data = deque(maxlen=200)
    target_y_data = deque(maxlen=200)

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8").strip()
                if line and "," in line:
                    try:
                        parts = line.split(",")
                        ball_x = float(parts[1])
                        if ball_x > 90: continue
                        ball_y = float(parts[2])
                        target_x = float(parts[3])
                        target_y = float(parts[4])

                        # Adds data to list
                        ball_x_data.append(ball_x)
                        ball_y_data.append(ball_y)
                        target_x_data.append(target_x)
                        target_y_data.append(target_y)

                        # Update plot
                        ax.clear()
                        ax.plot(ball_x_data, ball_y_data, "b-", label="Ball path")
                        ax.plot(target_x_data, target_y_data, "r--", label="Target path")
                        ax.set_xlabel("X position (mm)")
                        ax.set_ylabel("Y position (mm)")
                        ax.set_title("Ball Tracking Plotter")
                        ax.legend
                        ax.grid(True)
                        ax.set_aspect('equal')
                        plt.draw
                        plt.pause(0.0001)

                    except ValueError:
                        # If line is not in proper format or not read properly, line gets skipped
                        continue

    except KeyboardInterrupt:
        print("Stopping the plot")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

# Usage
ports = serial.tools.list_ports.comports()
print(ports)
real_time_plotter("COM5", 115200)


