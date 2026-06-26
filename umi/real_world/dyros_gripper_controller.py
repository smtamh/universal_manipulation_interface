import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.precise_sleep import precise_wait
from umi.real_world.wsg_binary_driver import WSGBinaryDriver
from umi.real_world.dyros_binary_driver import DYROSBinaryDriver
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from loguru import logger

# FARM
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._integral = 0
        self._last_error = 0
        self._last_time = None

    def update(self, measurement):
        current_time = time.time()
        if self._last_time is None:
            self._last_time = current_time
            return 0

        dt = current_time - self._last_time
        if dt <= 0:
            return 0 # 시간 변화가 없으면 계산 생략

        error = self.setpoint - measurement
        P = self.Kp * error
        self._integral += error * dt

        # (Anti-windup)
        if self.output_limits[0] is not None:
            self._integral = max(self._integral, self.output_limits[0] / (self.Ki + 1e-6))
        if self.output_limits[1] is not None:
            self._integral = min(self._integral, self.output_limits[1] / (self.Ki + 1e-6))
        I = self.Ki * self._integral
        
        derivative = (error - self._last_error) / dt
        D = self.Kd * derivative
        
        # PID 출력 계산
        output = P + I + D
        
        # 출력 제한
        if self.output_limits[0] is not None:
            output = max(output, self.output_limits[0])
        if self.output_limits[1] is not None:
            output = min(output, self.output_limits[1])
            
        self._last_error = error
        self._last_time = current_time
        
        return output

    def reset(self):
        self._integral = 0
        self._last_error = 0
        self._last_time = None

class Command(enum.Enum):
    SHUTDOWN = 0
    SCHEDULE_WAYPOINT = 1
    RESTART_PUT = 2

class DYROSController(mp.Process):
    def __init__(self,
            shm_manager: SharedMemoryManager,
            # hostname,
            # port=1000,
            frequency=30,
            home_to_open=True,
            move_max_speed=200.0,
            get_max_k=None,
            command_queue_size=1024,
            launch_timeout=3,
            receive_latency=0.0,
            use_meters=False,
            verbose=False,
            disable_torque_on_close=True
            ):
        super().__init__(name="DYROSController")
        # self.hostname = hostname
        # self.port = port
        self.frequency = frequency
        self.home_to_open = home_to_open
        self.move_max_speed = move_max_speed
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.scale = 1000.0 if use_meters else 1.0
        self.verbose = verbose
        self.disable_torque_on_close = disable_torque_on_close

        self.pid_controller = PID(Kp=0.001, Ki=0.00001, Kd=0.0)
        self.force_interpolator = None

        if get_max_k is None:
            get_max_k = int(frequency * 10)
        
        # build input queue
        example = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=command_queue_size
        )
        
        # build ring buffer
        example = {
            'gripper_position': 0.0,
            'gripper_velocity': 0.0,
            'gripper_receive_timestamp': time.time(),
            'gripper_timestamp': time.time()
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )
        
        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[dyrosController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.SHUTDOWN.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()
    
    def stop_wait(self):
        self.join()

    @property
    def is_ready(self):
        return self.ready_event.is_set()
    
    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        
    # ========= command methods ============
    def schedule_waypoint(self, pos: float, target_time: float):
        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': pos,
            'target_time': target_time
        }
        self.input_queue.put(message)


    def restart_put(self, start_time):
        self.input_queue.put({
            'cmd': Command.RESTART_PUT.value,
            'target_time': start_time
        })
    
    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()
    
    # ========= main loop in process ============
    def run(self):
        # start connection
        try:
            with DYROSBinaryDriver(
                disable_torque_on_close=self.disable_torque_on_close
                ) as dgripper:
                # get initial early state before long sleep to populate ring buffer
                print(f"[DEBUG] DYROSController: home_to_open={self.home_to_open}")
                try:
                    early_info = dgripper.script_query()
                    early_state = {
                        'gripper_position': early_info['position'] / self.scale,
                        'gripper_velocity': early_info['velocity'] / self.scale,
                        'gripper_receive_timestamp': time.time(),
                        'gripper_timestamp': time.time() - self.receive_latency
                    }
                    self.ring_buffer.put(early_state)
                    print("[DEBUG] DYROSController: Pushed early state to ring buffer.")
                except Exception as e:
                    print(f"[DEBUG] Warning: Failed to get early query: {e}")

                dgripper.ack_fault()
                if self.home_to_open:
                    print("[DEBUG] DYROSController: Executing homing()")
                    print("[DEBUG] DYROSController: Executing homing()")
                    try:
                        dgripper.homing()
                    except RuntimeError as e:
                        print(f"[DYROSController] Error during homing: {e}")
                        print("[DYROSController] Attempting to recover (ack_fault)...")
                        dgripper.ack_fault()
                else:
                    print("[DEBUG] DYROSController: Skipping homing()")
                time.sleep(2)       # control command 유지
                # get initial
                curr_info = dgripper.script_query()  # 상태 정보를 요청하여 반환
                curr_pos = curr_info['position']
                print(f"[DEBUG] DYROSController: Initial curr_pos={curr_pos}")
                # curr_pos = 100.0
                if not self.home_to_open: 
                    # print("[DEBUG] DYROSController: Overriding initial pos to 58.0")
                    # curr_pos = 58.0   # erase
                    # curr_pos = 60.0 # pull block
                    # curr_pos = 53.0 # pull ball
                    # curr_pos = 49.0 # pull ball - new motor
                    # curr_pos = 80.0 # transfer_fluid
                    # curr_pos = 36.0 # transfer_fluid
                    pass
                curr_t = time.monotonic()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(
                    times=[curr_t],
                    poses=[[curr_pos,0,0,0,0,0]]
                )
                
                keep_running = True
                t_start = time.monotonic()
                iter_idx = 0
                while keep_running:
                    # command gripper
                    t_now = time.monotonic()
                    dt = 1 / self.frequency
                    t_target = t_now
                    target_pos = pose_interp(t_target)[0]
                    # target_pos = target_pos - 5.0   # 08/07
                    # print(f"target_pos:{target_pos}")
                    target_vel = (target_pos - pose_interp(t_target - dt)[0]) / dt
                    # print('controller target pos & vel: ', target_pos, target_vel)

                    # 특정 위치(target_pos)와 속도(target_vel)를 목표로 그리퍼를 제어하는 명령을 하드웨어로 전송 -> dyros gripper는 오직 위치제어만
                    # 반환값 info는 명령 수행 결과에 대한 정보를 포함
                    # [Fixed] Error Recovery Logic
                    try:
                        info = dgripper.script_position_pd(
                            position=target_pos, velocity=int(self.move_max_speed))
                        
                        # [Added] Get Actual State Feedback
                        try:
                            actual_info = dgripper.script_query()
                            info = actual_info # Update with actual position/velocity
                        except Exception as e:
                             print(f"[DYROSController] Warning: Failed to query actual state: {e}")
                    except RuntimeError as e:
                        print(f"[DYROSController] Error sending command: {e}")
                        print("[DYROSController] Attempting to recover (ack_fault)...")
                        try:
                            dgripper.ack_fault()
                        except Exception as e2:
                            print(f"[DYROSController] Recovery failed: {e2}")
                        
                        # Use last known info or dummy
                        info = {'position': target_pos, 'velocity': 0.0}
                        time.sleep(0.05) # Wait a bit before retrying
                    # time.sleep(1e-3)

                    # get state from robot
                    raw1 = info.get('raw_pos_1', 0)
                    raw2 = info.get('raw_pos_2', 0)
                    print(f"[DEBUG] DYROS Target Width: {target_pos:.2f} mm | Current Width: {info['position']:.2f} mm | Raw1: {raw1} | Raw2: {raw2}")
                    state = {
                        'gripper_position': info['position'] / self.scale,
                        'gripper_velocity': info['velocity'] / self.scale,
                        'gripper_receive_timestamp': time.time(),
                        'gripper_timestamp': time.time() - self.receive_latency
                    }
                    self.ring_buffer.put(state)

                    # fetch command from queue
                    try:
                        commands = self.input_queue.get_all()
                        n_cmd = len(commands['cmd'])
                    except Empty:
                        n_cmd = 0
                        # print("n_cmd =0")
                    
                    # execute commands
                    # 큐에서 받은 명령을 해석하고 실행 (Shutdown 명령, 웨이포인트 명령, 경로 재설정)
                    # 명령 처리 부분은 local에서 돌아가는 로직 -> 제어 명령을 생성(목표 위치(target_pos)와 속도(target_vel)를 계산) -> info = wsg.script_position_pd(하드웨어와의 통신)
                    for i in range(n_cmd):
                        command = dict()
                        for key, value in commands.items():
                            command[key] = value[i]
                        cmd = command['cmd']
                        
                        if cmd == Command.SHUTDOWN.value:
                            keep_running = False
                            # stop immediately, ignore later commands
                            break
                        elif cmd == Command.SCHEDULE_WAYPOINT.value:
                            target_pos = command['target_pos'] * self.scale
                            target_time = command['target_time']
                            # translate global time to monotonic time
                            target_time = time.monotonic() - time.time() + target_time
                            curr_time = t_now
                            pose_interp = pose_interp.schedule_waypoint(
                                pose=[target_pos, 0, 0, 0, 0, 0],
                                time=target_time,
                                max_pos_speed=self.move_max_speed,
                                max_rot_speed=self.move_max_speed,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time
                            )
                            last_waypoint_time = target_time
                        elif cmd == Command.RESTART_PUT.value:
                            t_start = command['target_time'] - time.time() + time.monotonic()
                            iter_idx = 1
                        else:
                            keep_running = False
                            break
                        
                    # first loop successful, ready to receive command
                    if iter_idx == 0:
                        self.ready_event.set()
                    iter_idx += 1
                    
                    # regulate frequency
                    dt = 1 / self.frequency
                    t_end = t_start + dt * iter_idx
                    precise_wait(t_end=t_end, time_func=time.monotonic)
                
        except Exception as global_e:
            import traceback
            print(f"[DYROSController] FATAL ERROR IN RUN LOOP: {global_e}")
            traceback.print_exc()
        finally:
            self.ready_event.set()
            if self.verbose:
                print(f"[DYROSController] Disconnected from robot")