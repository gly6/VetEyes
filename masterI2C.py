import smbus
import time
import threading

bus = smbus.SMBus(1)

def sendData(data):
    slaveAddress = 0x28
    intsOfData = list(map(ord, data))
    # print(intsOfData)
    bus.write_i2c_block_data(slaveAddress, intsOfData[0], intsOfData[1:])

def readData():
    slaveAddress = 0x28
    while True:
        try:
            data = bus.read_i2c_block_data(slaveAddress, 0x00)
            # print(data)
            if data[0] != 0:
                res = ''.join(map(chr, data))
                # print(len(res))
                print("recieve from slave", res)


        except:
            print("remote i/o erroe")
            
        time.sleep(1)


# readData()

# i2c = Serial_input(9600)
T_i2C = threading.Thread(target=readData)
T_i2C.start()

