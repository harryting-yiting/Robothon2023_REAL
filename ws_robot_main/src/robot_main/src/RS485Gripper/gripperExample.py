from rs485gp import RS485BusGripper
import time

# Create a gripper object
Gripper = RS485BusGripper(name='RS485Gripper',device='/dev/ttyUSB0', RSAddress=1,Baud = 115200)

# Initialize Gripper
Gripper.InitializeGripper()

time.sleep(3)

# bool: Indicating if the gripper is initialized and ready
startUpComplete = Gripper.GetInitializeStatus()
print('Initialized: ',startUpComplete)

# Set 30% Grip force
Gripper.SetForceLimitPercentage_20_100(30)

# Set 50% Grip force
Gripper.SetSpeedLimitPercentage_1_100(50)

# Go to position 500, (0-1000)
Gripper.SetPosition_0_1000(500)
time.sleep(1.5)

# Go to position 500, (0-1000)
Gripper.SetPosition_0_1000(1000)
time.sleep(1.5)

# Go to position 500, (0-1000)
Gripper.SetPosition_0_1000(500)
time.sleep(1.5)

# Read the grasp status of the gripper
# 'Moving':     Gripper moving
# 'NoObject':   No griping load detected
# 'Grasped':    Object grip force detected
# 'Object Fall':No grip force when no release action is done
objectGrasp = Gripper.ReadGraspState()
print('Grasp status: ',objectGrasp)

# Close COM Port
Gripper.Quit()
