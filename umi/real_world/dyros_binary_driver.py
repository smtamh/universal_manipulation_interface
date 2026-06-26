# # import rclpy
# # from rclpy.node import Node
# from dynamixel_sdk import PortHandler, PacketHandler
# # from std_msgs.msg import Float32

# # Motor and communication settings
# DEVICENAME = "/dev/ttyUSB0"
# PROTOCOL_VERSION = 2
# BAUDRATE = 57600

# # Dynamixel XL330 memory addresses
# CURRENT_BASED_POSITION_MODE = 5
# ADDR_XL330_OPERATING_MODE = 11
# ADDR_XL330_TORQUE_ENABLE = 64
# ADDR_XL330_GOAL_POSITION = 116
# ADDR_XL330_CURRENT_LIMIT = 38
# ADDR_XL330_GOAL_CURRENT = 102
# TORQUE_ENABLE = 1
# TORQUE_DISABLE = 0

# ADDR_XL330_PRESENT_VELOCITY = 128
# ADDR_XL330_PRESENT_POSITION = 132
# # Current and position limits
# CURRENT_LIMIT = 200
# GOAL_CURRENT = 160
# # CURRENT_LIMIT = 500 # change
# # GOAL_CURRENT = 450  # change

# # GOAL_POSITION_OPEN = 2048
# # GOAL_POSITION_CLOSE = 1024
# # GOAL_POSITION_OPEN = 1024
# # GOAL_POSITION_CLOSE = 2048
# # Motor 1 Limits
# # ID1_POSITION_OPEN = 921
# # ID1_POSITION_CLOSE = 1911
# ID1_POSITION_OPEN = 996
# ID1_POSITION_CLOSE = 1940

# # Motor 2 Limits
# ID2_POSITION_OPEN = 3144
# ID2_POSITION_CLOSE = 2053
# # ID2_POSITION_OPEN = 3313
# # ID2_POSITION_CLOSE = 2312


# # GRIPPER_MAX_WIDTH =110 # reference from wsg_50 gripper width 
# GRIPPER_MAX_WIDTH =85 

# # Motor IDs
# DXL_ID_1 = 1
# DXL_ID_2 = 2
# # DXL_ID_1 = 3
# # DXL_ID_2 = 4
# COMM_SUCCESS = 0

# class DYROSBinaryDriver:
#     def __init__(self, disable_torque_on_close=True):
#         self.portHandler = PortHandler(DEVICENAME)
#         self.packetHandler = PacketHandler(PROTOCOL_VERSION)
#         self.disable_torque_on_close = disable_torque_on_close

#         # Open port and set baudrate
#         if not self.portHandler.openPort():
#             raise RuntimeError("Failed to open the port")
#         if not self.portHandler.setBaudRate(BAUDRATE):
#             raise RuntimeError("Failed to set the baudrate")

#         # Initialize motors
#         self.init_motors()
#         print("init_motors")

#     def __enter__(self):
#         return self
    
#     def __exit__(self, exc_type, exc_val, exc_tb):
#         self.close()

#     def init_motors(self):
#         # Motor 1 initialization
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
#         self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_CURRENT_LIMIT, CURRENT_LIMIT)
#         self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_GOAL_CURRENT, GOAL_CURRENT)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE)

#         # Motor 2 initialization
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
#         self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_CURRENT_LIMIT, CURRENT_LIMIT)
#         self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_GOAL_CURRENT, GOAL_CURRENT)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE)

#     def set_goal_positions(self, pos1: int, pos2: int):
#         """Set goal positions for the gripper motors independently"""
#         result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_GOAL_POSITION, pos1)
#         if result != COMM_SUCCESS or error != 0:
#             raise RuntimeError(f"Motor ID {DXL_ID_1} error: {self.packetHandler.getTxRxResult(result)} | RxPacketError: {self.packetHandler.getRxPacketError(error)}")

#         result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_GOAL_POSITION, pos2)
#         if result != COMM_SUCCESS or error != 0:
#             raise RuntimeError(f"Motor ID {DXL_ID_2} error: {self.packetHandler.getTxRxResult(result)} | RxPacketError: {self.packetHandler.getRxPacketError(error)}")

#     def ack_fault(self):
#         """Acknowledge and reset any fault conditions"""
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
#         self.init_motors()

#     def homing(self):
#         """Move the gripper to the open position"""
#         self.set_goal_positions(ID1_POSITION_OPEN, ID2_POSITION_OPEN)

#     def script_query(self):
#         """Query the current position and state of the gripper"""
#         raw_position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_PRESENT_POSITION)
#         velocity, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_PRESENT_VELOCITY)

#         # Scale the raw position to match the WSG gripper width range -> gripper width scaled position(0-110)
#         scaled_position = ((raw_position - ID1_POSITION_CLOSE) / (ID1_POSITION_OPEN - ID1_POSITION_CLOSE)) * GRIPPER_MAX_WIDTH
        
#         # Read Motor 2 position as well for debugging
#         raw_position2, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_PRESENT_POSITION)
        
#         info = {
#             'position': scaled_position,
#             'velocity': velocity,
#             'raw_pos_1': raw_position,
#             'raw_pos_2': raw_position2
#         }
        
#         return info

#     def script_position_pd(self, position: float, velocity: float):
#         """Set position and velocity for the gripper"""
#         # Calculate interpolation factor based on target width (position)
#         factor = position / GRIPPER_MAX_WIDTH
#         factor = max(0.0, min(1.0, factor))
        
#         # Scale position to Dynamixel ranges
#         scaled_pos1 = int(ID1_POSITION_CLOSE + (ID1_POSITION_OPEN - ID1_POSITION_CLOSE) * factor)
#         scaled_pos2 = int(ID2_POSITION_CLOSE + (ID2_POSITION_OPEN - ID2_POSITION_CLOSE) * factor)

#         # 속도 제한 적용
#         self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_PRESENT_VELOCITY, int(velocity))
#         self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_PRESENT_VELOCITY, int(velocity))

#         self.set_goal_positions(scaled_pos1, scaled_pos2)

#         info = {
#             'position': position,
#             'velocity': velocity,
#             'raw_pos_1': scaled_pos1,
#             'raw_pos_2': scaled_pos2
#         }
#         return info


#     def disable_torque(self):
#         """Disable motor torque"""
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)

#     def enable_torque(self):
#         """Enable motor torque"""
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE)
#         self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE)

#     def close(self):
#         """Release resources and close the port"""
#         if self.disable_torque_on_close:
#             self.disable_torque()
#         self.portHandler.closePort()

# def test():
#     import numpy as np
#     import time

#     # Dynamixel 드라이버 초기화
#     driver = DYROSBinaryDriver()
    
#     try:
#         # 초기화 및 Fault Acknowledge
#         print("Acknowledging faults...")
#         driver.ack_fault()
        
#         # Homing (Gripper 열림)
#         print("Homing gripper...")
#         driver.homing()
#         time.sleep(1)

#         # 테스트: 열림 -> 닫힘 -> 열림
#         print("Testing open-close-open sequence...")
#         T = 2  # 각 동작 지속 시간 (초)
#         dt = 1 / 30  # 제어 주기 (초) - 30Hz

#         # 열림 -> 닫힘
#         print("Moving from open to close...")
#         pos_open_to_close = np.linspace(GRIPPER_MAX_WIDTH, 0., int(T / dt))
#         for target_position in pos_open_to_close:
#             print(f"Target Position: {target_position:.2f} mm")
#             driver.script_position_pd(position=target_position, velocity=50)

#             info = driver.script_query()
#             print(f"Gripper Position: {info['position']:.2f} mm, Gripper Velocity: {info['velocity']}")

#             time.sleep(dt)

#         # 닫힘 -> 열림
#         print("Moving from close to open...")
#         pos_close_to_open = np.linspace(0., GRIPPER_MAX_WIDTH, int(T / dt))
#         for target_position in pos_close_to_open:
#             print(f"Target Position: {target_position:.2f} mm")
#             driver.script_position_pd(position=target_position, velocity=50)

#             info = driver.script_query()
#             print(f"Gripper Position: {info['position']:.2f} mm, Gripper Velocity: {info['velocity']}")

#             time.sleep(dt)

#         print("Test completed.")

#     except Exception as e:
#         print(f"An error occurred: {e}")

#     finally:
#         # 드라이버 종료 및 리소스 해제
#         print("Closing driver...")
#         driver.close()
    
# def grasp():
#     import numpy as np
#     import time
#     driver = DYROSBinaryDriver()
#     T = 2  # 각 동작 지속 시간 (초)
#     dt = 1 / 30
#     print("Homing gripper...")
#     driver.homing()
#     time.sleep(1)
#     close = False
#     while True:
#         if close:
#             driver.script_position_pd(position=60, velocity=10)
#             print("Moving from open to close...")
#         else:
#             pos_open_to_close = np.linspace(GRIPPER_MAX_WIDTH, 55, int(T / dt))
#             for target_position in pos_open_to_close:
#                 print(f"Target Position: {target_position:.2f} mm")
#                 driver.script_position_pd(position=target_position, velocity=50)

#                 info = driver.script_query()
#                 print(f"Gripper Position: {info['position']:.2f} mm, Gripper Velocity: {info['velocity']}")
#                 time.sleep(dt)
#                 close = True


# # PID 게인 설정 함수
# def set_pid_gains(portHandler, packetHandler, dxl_id, p_gain, i_gain, d_gain):
#     # Proportional Gain
#     result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, 84, int(p_gain))
#     if result != COMM_SUCCESS or error != 0:
#         raise RuntimeError(f"Failed to set P-Gain: {packetHandler.getTxRxResult(result)}")

#     # Integral Gain
#     result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, 82, int(i_gain))
#     if result != COMM_SUCCESS or error != 0:
#         raise RuntimeError(f"Failed to set I-Gain: {packetHandler.getTxRxResult(result)}")

#     # Derivative Gain
#     result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, 80, int(d_gain))
#     if result != COMM_SUCCESS or error != 0:
#         raise RuntimeError(f"Failed to set D-Gain: {packetHandler.getTxRxResult(result)}")



# def main(args=None):
#     # Example usage
#     driver = DYROSBinaryDriver()
#     set_pid_gains(driver.portHandler, driver.packetHandler, DXL_ID_1, p_gain=15, i_gain=0, d_gain=3)
#     set_pid_gains(driver.portHandler, driver.packetHandler, DXL_ID_2, p_gain=15, i_gain=0, d_gain=3)
#     try:
#         # test 함수 실행
#         # test()
#         grasp()
#     except KeyboardInterrupt:
#         print("Test interrupted by user.")
    


# if __name__ == '__main__':
#     main()

#################################################################################### XM540-W270-R 버전으로 업데이트

# import rclpy
# from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
# from std_msgs.msg import Float32

# Motor and communication settings
DEVICENAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2
BAUDRATE = 57600

# Dynamixel XM540-W270-R memory addresses
CURRENT_BASED_POSITION_MODE = 5
ADDR_DXL_OPERATING_MODE = 11
ADDR_DXL_DRIVE_MODE = 10
ADDR_DXL_TORQUE_ENABLE = 64
ADDR_DXL_GOAL_POSITION = 116
ADDR_DXL_CURRENT_LIMIT = 38
ADDR_DXL_GOAL_CURRENT = 102
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

ADDR_DXL_PRESENT_VELOCITY = 128
ADDR_DXL_PRESENT_POSITION = 132
# Current and position limits
# XM540-W270-R: 1 unit = 2.69mA. 
# Max Current Limit = 2047 (~5.5A).
# CURRENT_LIMIT = 2047 
# Reduced for safety testing (increased slightly from 200 to 300)
CURRENT_LIMIT = 400
# Goal Current for gripping: 1000 (~2.7A)
# GOAL_CURRENT = 1000
# Reduced for safety testing
GOAL_CURRENT = 200


# Calibrated Limits (Close, Open)
ID1_RANGE = (1562, 2435)
ID2_RANGE = (1778, 2736)


GRIPPER_MAX_WIDTH = 85 # mm

# Motor IDs
DXL_ID_1 = 1
DXL_ID_2 = 2
COMM_SUCCESS = 0

class DYROSBinaryDriver:
    def __init__(self, disable_torque_on_close=True):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.disable_torque_on_close = disable_torque_on_close

        # Open port and set baudrate
        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open the port")
        if not self.portHandler.setBaudRate(BAUDRATE):
            raise RuntimeError("Failed to set the baudrate")

        # Initialize motors
        self.init_motors()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def init_motors(self):
        # Motor 1 initialization (Normal Mode)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_DRIVE_MODE, 0) # Normal Mode
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_CURRENT_LIMIT, CURRENT_LIMIT)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_GOAL_CURRENT, GOAL_CURRENT)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_TORQUE_ENABLE, TORQUE_ENABLE)

        # Motor 2 initialization (Reverse Mode)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_DRIVE_MODE, 1) # Reverse Mode
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_CURRENT_LIMIT, CURRENT_LIMIT)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_GOAL_CURRENT, GOAL_CURRENT)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_TORQUE_ENABLE, TORQUE_ENABLE)

    def get_target_positions(self, width_mm: float):
        """Calculate target positions for both motors based on width (mm)"""
        # Clamp width
        width_mm = max(0.0, min(GRIPPER_MAX_WIDTH, width_mm))
        factor = width_mm / GRIPPER_MAX_WIDTH
        
        # Interpolate ID 1
        pos1 = int(ID1_RANGE[0] + (ID1_RANGE[1] - ID1_RANGE[0]) * factor)
        
        # Interpolate ID 2
        pos2 = int(ID2_RANGE[0] + (ID2_RANGE[1] - ID2_RANGE[0]) * factor)
        
        return pos1, pos2

    def set_goal_position(self, position: int):
        """
        Legacy method signature, but 'position' here implies raw DXL position?
        Actually, usually this driver keeps state. 
        But let's assume the caller passes a target DXL position that was scaled elsewhere?
        Wait, in the original code, `script_position_pd` called `set_goal_position` with `scaled_position`.
        And `homing` called it with `GOAL_POSITION_OPEN`.
        
        Refactoring:
        If `position` is passed as a generic integer roughly around 0-4096, 
        it's hard to distinguish if it's for Motor 1 or 2 if they are different.
        
        However, `homing()` calls `set_goal_position(GOAL_POSITION_OPEN)`.
        I should update `homing` to use `width`.
        """
        # Warning: This direct setter is dangerous if motors have different ranges.
        # We will assume this is ONLY used for legacy raw integer writes, 
        # but let's redirect to safer logic if possible.
        # For now, just write to both, BUT this is likely wrong if called directly.
        
        # Better: let's rewrite `homing` and `script_position_pd` to use width logic,
        # and make `set_goal_position` accept specific targets for 1 and 2.
        pass 

    def set_goal_positions(self, pos1: int, pos2: int):
        """Set goal position for both motors"""
        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_GOAL_POSITION, pos1)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(f"Motor ID {DXL_ID_1} error: {self.packetHandler.getTxRxResult(result)}")

        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_GOAL_POSITION, pos2)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(f"Motor ID {DXL_ID_2} error: {self.packetHandler.getTxRxResult(result)}")

    def ack_fault(self):
        """Acknowledge and reset any fault conditions"""
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_TORQUE_ENABLE, TORQUE_DISABLE)
        self.init_motors()

    def homing(self):
        """Move the gripper to the open position"""
        pos1, pos2 = self.get_target_positions(GRIPPER_MAX_WIDTH)
        self.set_goal_positions(pos1, pos2)

    def script_query(self):
        """Query the current position and state of the gripper"""
        raw_position1, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_PRESENT_POSITION)
        velocity, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_PRESENT_VELOCITY)

        # Scale based on Motor 1 (Primary)
        # 0 width = ID1_RANGE[0], Max width = ID1_RANGE[1]
        raw_min = ID1_RANGE[0]
        raw_max = ID1_RANGE[1]
        
        scaled_position = ((raw_position1 - raw_min) / (raw_max - raw_min)) * GRIPPER_MAX_WIDTH
        
        info = {
            'position': scaled_position,
            'velocity': velocity,
        }
        
        return info

    def script_position_pd(self, position: float, velocity: float):
        """Set position (width in mm) and velocity for the gripper"""
        
        pos1, pos2 = self.get_target_positions(position)

        # 속도 제한 적용
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_PRESENT_VELOCITY, int(velocity))
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_PRESENT_VELOCITY, int(velocity))

        self.set_goal_positions(pos1, pos2)
        
        info = {
            'position': position, # commanded position
            'velocity': velocity,
        }
        return info


    def disable_torque(self):
        """Disable motor torque"""
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_1, ADDR_DXL_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_2, ADDR_DXL_TORQUE_ENABLE, TORQUE_DISABLE)

    def close(self):
        """Release resources and close the port"""
        if self.disable_torque_on_close:
            self.disable_torque()
        self.portHandler.closePort()

def test():
    import numpy as np
    import time

    # Dynamixel 드라이버 초기화
    driver = DYROSBinaryDriver()
    
    try:
        # 초기화 및 Fault Acknowledge
        print("Acknowledging faults...")
        driver.ack_fault()
        
        # Homing (Gripper 열림)
        print("Homing gripper...")
        driver.homing()
        time.sleep(1)

        # 테스트: 열림 -> 닫힘 -> 열림
        print("Testing open-close-open sequence...")
        T = 2  # 각 동작 지속 시간 (초)
        dt = 1 / 30  # 제어 주기 (초) - 30Hz

        # 열림 -> 닫힘
        print("Moving from open to close...")
        pos_open_to_close = np.linspace(GRIPPER_MAX_WIDTH, 0., int(T / dt))
        for target_position in pos_open_to_close:
            print(f"Target Position: {target_position:.2f} mm")
            driver.script_position_pd(position=target_position, velocity=50)

            info = driver.script_query()
            print(f"Gripper Position: {info['position']:.2f} mm, Gripper Velocity: {info['velocity']}")

            time.sleep(dt)

        # 닫힘 -> 열림
        print("Moving from close to open...")
        pos_close_to_open = np.linspace(0., GRIPPER_MAX_WIDTH, int(T / dt))
        for target_position in pos_close_to_open:
            print(f"Target Position: {target_position:.2f} mm")
            driver.script_position_pd(position=target_position, velocity=50)

            info = driver.script_query()
            print(f"Gripper Position: {info['position']:.2f} mm, Gripper Velocity: {info['velocity']}")

            time.sleep(dt)

        print("Test completed.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # 드라이버 종료 및 리소스 해제
        print("Closing driver...")
        driver.close()

# PID 게인 설정 함수
def set_pid_gains(portHandler, packetHandler, dxl_id, p_gain, i_gain, d_gain):
    # Proportional Gain
    result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, 84, int(p_gain))
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Failed to set P-Gain: {packetHandler.getTxRxResult(result)}")

    # Integral Gain
    result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, 82, int(i_gain))
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Failed to set I-Gain: {packetHandler.getTxRxResult(result)}")

    # Derivative Gain
    result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, 80, int(d_gain))
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Failed to set D-Gain: {packetHandler.getTxRxResult(result)}")



def main(args=None):
    # Example usage
    driver = DYROSBinaryDriver()
    set_pid_gains(driver.portHandler, driver.packetHandler, DXL_ID_1, p_gain=15, i_gain=0, d_gain=3)
    set_pid_gains(driver.portHandler, driver.packetHandler, DXL_ID_2, p_gain=15, i_gain=0, d_gain=3)
    try:
        # test 함수 실행
        test()
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    


if __name__ == '__main__':
    main()
