from rs485gp import RS485BusGripper

Gripper = RS485BusGripper(name='RS485-1')

while 1:
    try:

        inputStr = input("Type in command: ")
        strlist = inputStr.split('=')
        strlist[0] = strlist[0].lower()
        if len(strlist) == 2:
            if strlist[0].strip() == 'move':
                num = float(strlist[1])
                Gripper.SetPosition_0_1000(num)
            if strlist[0].strip() == 'speed':
                num = float(strlist[1])
                Gripper.SetSpeedLimitPercentage_1_100(speed=num)
            elif strlist[0].strip() == 'force':
                num = float(strlist[1])
                Gripper.SetForceLimitPercentage_20_100(force=num)
            elif strlist[0].strip() == 'positiont':
                num = float(strlist[1])
                Gripper.ToPositionTurn(position=num)
            elif strlist[0].strip() == 'position':
                num = float(strlist[1])
                Gripper.ToPositionMM(positionMM=num)
            elif strlist[0].strip() == 'id':
                num = float(strlist[1])
                Gripper.motorAddress = num
                print('Motor communication ID changed to ' + str(num))
        elif len(strlist) == 1: 
            if strlist[0].strip() == 'initial':
                print(Gripper.InitializeGripper())
            elif strlist[0].strip() == 'initialize':
                print(Gripper.InitializeGripper())
            elif strlist[0].strip() == 'initialize?':
                print(Gripper.GetInitializeStatus())
            elif strlist[0].strip() == 'config':
                Gripper.ReadConfig()
            elif strlist[0].strip() == 'stop':
                Gripper.Stop()
            elif (strlist[0].strip() == 'exit' or strlist[0].strip() == 'quit'):
                Gripper.Quit()
                break
            elif (strlist[0].strip() == 'sitrep' or strlist[0].strip() == 'state'or strlist[0].strip() == 'scan'):
                Gripper.ReadState()
            elif (strlist[0].strip() == 'loop'):
                for i in range(50):
                    Gripper.ReadState()
            elif strlist[0].strip() == 'home':
                Gripper.Homing()
        line = Gripper.serial.readline()
        print('Serial receive: ',end = '')
        for bitt in line:
            print('%#x' % bitt, end=' ')
        print()
    except Exception as e:
        print("!!!!! Warning !!!!! : ", e)
        print('The second argument must be a number!!!')

