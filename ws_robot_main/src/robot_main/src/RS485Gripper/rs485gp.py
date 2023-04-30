# !!! Using pyserial package
# !!! 使用pyserial包
import serial
from serial.serialutil import STOPBITS_TWO
import serial.tools.list_ports
import time
import struct
import numpy



class RS485BusGripper:
    # TODO: None
    # 类名：RS485BusGripper
    # 成员：
    # name:        电机名称，暂无意义
    ## serial:     pyserial串口对象
    # !!!Default device: '/dev/ttyUSB0'
    def __init__(self, name='Gripper', device='/dev/ttyUSB0', RSAddress=1,Baud = 115200):
        # 初始化RS485总线对象
        # device:       可以传参命名为固定串口如'COM3'(Windows),'/dev/ttyUSB0'(Linux)；若不提前传参，会在控制台终端询问串口号(string)
        # name:         名称，暂无意义(string)
        # RSAddress:    夹爪在RS485总线上的地址(int)，默认为1
        # Baud:         Baud rate
        # By default, no parity correction, 1 stop bit.
        
        self.name = name
        self.device = device
        self.RSAddress = RSAddress
        self.Baud = Baud
        self.RS485_init()

    def RS485_init(self):
        # 初始化电机
        portsName = []
        try:
            port_list = list(serial.tools.list_ports.comports())
            print(port_list)
            if len(port_list) == 0:
                print('No serial port available')
                exit(1)
            else:
                for i in range(0, len(port_list)):
                    print(port_list[i])
                    portsName.append(port_list[i].device)
            while 1:
                if self.device != 'COM-1':
                    break
                inputStr = input(
                    "Type in serial port number: (Example: type in COM5 -> Enter): ")
                inputStr = inputStr.strip()
                for portName in portsName:
                    if inputStr == portName:
                        self.device = inputStr

            self.serial = serial.Serial(self.device, self.Baud, timeout=0.05)
            print("Serial Port Info: ", self.serial)

        except Exception as e:
            print("!!!!! Warning !!!!! : ", e)
            exit(1)

    def InitializeGripper(self):
        feedback = self.Write(RSAddress=self.RSAddress, regAddress=[0x01,0x00],value = [0x00,0x01],CRC=True)
        signal = [(int)(self.RSAddress)&0xff,0x06,0x01,0x00,0x00,0x01]
        signal = RS485BusGripper.ModRTU_CRC16(signal=signal)
        signal = bytearray(feedback)
        if (feedback == signal):
            return True
        else:
            return False

    def SetForceLimitPercentage_20_100(self,force = 20):
        if force < 20:
            force = 20
        if force > 100:
            force = 100

        forceHex = [0x00]
        force = (int)(force)
        forceHex.append(force & 0xff)

        feedback = self.Write(RSAddress=self.RSAddress, regAddress=[0x01,0x01],value = forceHex,CRC=True)

    def SetPosition_0_1000(self,position = 1000):
        if position < 0:
            position = 0
        if position > 1000:
            position = 1000

        positionHex = []
        position = (int)(position)
        positionHex.append((position>>8) & 0xff)
        positionHex.append(position & 0xff)

        feedback = self.Write(RSAddress=self.RSAddress, regAddress=[0x01,0x03],value = positionHex,CRC=True)

    
    def SetSpeedLimitPercentage_1_100(self,speed = 50):
        if speed < 1:
            speed = 1
        if speed > 100:
            speed = 100

        speedHex = [0x00]
        speed = (int)(speed)
        speedHex.append(speed & 0xff)

        feedback = self.Write(RSAddress=self.RSAddress, regAddress=[0x01,0x04],value = speedHex,CRC=True)

    def GetInitializeStatus(self):
        feedback = self.Read(RSAddress=self.RSAddress, regAddress=[0x02,0x00],value = [0x00,0x01],CRC=True)
        if (int)(feedback[4]==1):
            return True
        else:
            return False


    def ReadGraspState(self):
        feedback = self.Read(RSAddress=self.RSAddress, regAddress=[0x02,0x01],value = [0x00,0x01],CRC=True)
        if (int)(feedback[4]==0):
            return 'Moving'
        elif (int)(feedback[4]==1):
            return 'NoObject'
        elif (int)(feedback[4]==2):
            return 'Grasped'
        elif (int)(feedback[4]==3):
            return 'ObjectFall'
        
        

    def Write(self,RSAddress = 0x01,regAddress = [0x00,0x00],value = [0x00,0x00],CRC = True):
        signal = [(int)(RSAddress)&0xff, 0x06, regAddress[0],regAddress[1],value[0],value[1]]
        if CRC:
            signal = RS485BusGripper.ModRTU_CRC16(signal=signal)
        self.transmitSignal(signal=signal)
        return self.Receive()

    def Read(self,RSAddress = 0x01,regAddress = [0x00,0x00],value = [0x00,0x00],CRC = True):
        signal = [(int)(RSAddress)&0xff, 0x03, regAddress[0],regAddress[1],value[0],value[1]]
        if CRC:
            signal = RS485BusGripper.ModRTU_CRC16(signal=signal)
        self.transmitSignal(signal=signal)
        return self.Receive()

    def Receive(self):
        line = self.serial.read(40)
        print('RS485 receive: ',end = '')
        for bitt in line:
            print('%#x' % bitt, end=' ')
        print()
        return line


    def ModRTU_CRC16(signal):
        # ModBusRTU校验码算法函数
        leng = len(signal)
        crc = 0xffff
        crcSeq = []
        for pos in range(leng):
            crc ^= signal[pos]
            crcSeq.append(signal[pos])
            for i in range(8, 0, -1):
                if ((crc & 0x0001) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        crcSeq.append(crc & 0xff)
        crcSeq.append(crc >> 8)
        return crcSeq

    def transmitSignal(self, signal):
        print('RS485 transmmit: ', end='')
        # 串口发送16进制信号
        for sig in signal:
            print('%#x' % sig, end=' ')
        print()
        if self.serial.writable():
            self.serial.write(signal)

    def floatToBytes(self, f):
        bs = struct.pack("f", f)
        return (bs[3], bs[2], bs[1], bs[0])

    
    def Quit(self):
        # 切断与电机的通信
        self.serial.close()
