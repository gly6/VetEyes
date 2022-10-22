import serial
from collections import deque
import time
import threading

class Serial_input:
    def __init__(self, baud = 9600):
        self.ser = serial.Serial("/dev/ttyACM0", baud, timeout=1)
        self.ser.baudrate = baud
        self.queue = deque()
        self.ser.reset_input_buffer()

    def getSerialString(self):
        while(True):
                
            # print("start sound detect while loop")
            input1= self.ser.readline()
            if len(input1) > 3 and input1[0] == 99:
                temp_read = input1.decode('utf-8').strip()
                # temp_read = temp_read.strip()
                self.queue.append(temp_read)
                print("get from serial", temp_read)
                time.sleep(1)
    
    def sendSerialString(self, sendStr):
        sendStr += "\n"
        output1 = sendStr.encode('utf-8')
        self.ser.write(output1)

# serial_input = Serial_input()

# T_soundDetect = threading.Thread(target=serial_input.getSerialString)
# T_soundDetect.start()
    
# while(True):
#     time.sleep(2)
#     if serial_input.queue:
#         input1 = serial_input.queue.popleft()
#         inputList = input1.split("_")
#         print(inputList)
#         if inputList[1] == "mode":
#             if inputList[2] == "general":
#                 print("general")
#                 serial_input.sendSerialString("change mode, general")
#             elif inputList[2] == "clinical":
#                 print("clinical")
#                 serial_input.sendSerialString("change mode, clinical")

#         elif inputList[1] == "pupilsize":
#             temp_size = int(inputList[2])
#             serial_input.sendSerialString("change pupil size, " + str(temp_size))

#         elif inputList[1] == "pos":
#             try:
#                 posX, posY = int(inputList[2]), int(inputList[3])
#                 print(posX, posY)
#                 serial_input.sendSerialString("change pos, " + str(posX) + ", " + str(posY))
#             except:
#                 pass