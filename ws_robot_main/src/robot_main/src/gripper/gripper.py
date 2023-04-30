#!/usr/bin/env python

import sys, tty, termios
from scservo_sdk import *                      # Uses SCServo SDK library




class gripper_ctrl():
    def __init__(self,ID=7,DEVICE='/dev/ttyUSB0'):
        global SCS_ID,BAUDRATE,DEVICENAME,SCS_MINIMUM_POSITION_VALUE,SCS_MAXIMUM_POSITION_VALUE,SCS_MOVING_SPEED,SCS_MOVING_ACC
        # Default setting
        SCS_ID                      = ID                 # SCServo ID : 1
        BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
        DEVICENAME                  = DEVICE    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        SCS_MINIMUM_POSITION_VALUE  = 1720           # SCServo will rotate between this value
        SCS_MAXIMUM_POSITION_VALUE  = 2050
        SCS_MOVING_SPEED            = 3000        # SCServo moving speed
        SCS_MOVING_ACC              = 200          # SCServo moving acc

        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        sys.path.append("..")
        self.scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        self.packetHandler = sms_sts(self.portHandler)
        
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()
        
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()
        # 把后边可能用到的 sub, pub 在初始化函数中定义好

    def getch(self):
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return ch
    
    def callback(self, msg):
        if (msg.data == "G0"):
            self.grasp()
        elif (msg.data == "G1"):
            self.release()
            
    def grasp(self):
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(SCS_ID, self.scs_goal_position[0], SCS_MOVING_SPEED, SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
            
    def release(self):
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(SCS_ID, self.scs_goal_position[1], SCS_MOVING_SPEED, SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
            self.packetHandler.WritePosEx(SCS_ID, self.scs_goal_position[1], SCS_MOVING_SPEED, SCS_MOVING_ACC)
            
    def close(self):
        self.portHandler.closePort()


if __name__ == '__main__':
    gripper_ctrl(7,'/dev/ttyUSB0')
