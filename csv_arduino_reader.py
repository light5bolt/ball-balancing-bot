import serial.tools.list_ports
import serial
import time
import csv

ports = serial.tools.list_ports.comports()

# Selects serial port and creates new serialCom
port_list = []
for port in ports:
    port_list.append(str(port))
    print(str(port))

f = open("data.csv", "w", newline='')
f.truncate()

val = input("Select Port: COM")

for x in range(0, len(port_list)):
    if port_list[x].startswith("COM" + str(val)):
        port_var = "COM" + str(val)
        print(f"Selected: {port_list[x]}")
        break

if not port_var:
    print(f"Port COM{val} not found!")
    exit()


# Create and configure serial connection
try:
    serialCom = serial.Serial()
    serialCom.baudrate = 115200
    serialCom.port = port_var
    serialCom.timeout = 2  # Add timeout
    
    # Open the connection
    serialCom.open()
    print(f"Connected to {port_var}")

    # Resets Arduino
    serialCom.setDTR(False)
    time.sleep(1)
    serialCom.flushInput()
    serialCom.setDTR(True)

    kmax = 100
    for k in range(kmax):
        try:
            #Reads lines of data
            s_bytes = serialCom.readline()
            #Decode binary
            decoded_bytes = s_bytes.decode("utf-8").rstrip("\n")
            # print(decoded_bytes)

            # Parsing lines
            if k == 0:
                values = decoded_bytes.split(",")
            else:
                values = [float(x) for x in decoded_bytes.split(",")]
            
            print(values)

            writer = csv.writer(f, delimiter=",")
            writer.writerow(values)

        except:
            print("ERROR! Line was not recorded")

except Exception as e:
    print(f"Connection error: {e}")

f.close() #Close csv file