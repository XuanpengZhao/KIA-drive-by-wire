import time
from can_connection import CANConnection
from constant import Constant

class SteeringControl:
    def __init__(self, can_connection):
        self._can_connection = can_connection
        self._TORQUE_SUM = 3085
        self._TORQUE_PLUS_ZERO = 1551
        self._TORQUE_MINUS_ZERO = 1534  # Adjusted to ensure sum is 3085
        self._TORQUE_RANGE = 220
        self._last_running_message_time = 0

        # Register the callback for receiving messages
        self._can_connection.add_listener(self.receive_message)

    def set_torque(self, torque_value):
        # Convert torque value from -1 to 1 range to actual CAN values
        torque_plus = int(self._TORQUE_PLUS_ZERO + torque_value * self._TORQUE_RANGE)
        torque_minus = self._TORQUE_SUM - torque_plus
        data = [
            (torque_plus >> 24) & 0xFF, (torque_plus >> 16) & 0xFF, (torque_plus >> 8) & 0xFF, torque_plus & 0xFF,
            (torque_minus >> 24) & 0xFF, (torque_minus >> 16) & 0xFF, (torque_minus >> 8) & 0xFF, torque_minus & 0xFF
        ]
        print("Steering torque sent: ", torque_plus, ", ", torque_minus)
        self._can_connection.send_message(Constant.STM32_STEERING_TORQUE, data)
    
    def emergency_stop(self, stopOrReset):
        if stopOrReset == "STOP":
            self._can_connection.send_message(Constant.STM32_STEERING_ESTOP, b'STOP')
        elif stopOrReset == "RESET":
            self._can_connection.send_message(Constant.STM32_STEERING_ESTOP, b'RESET')

    def receive_message(self, message):
        if message.arbitration_id == Constant.STM32_STEERING_ANGLE:  # Receive angle messages
            if len(message.data) == 8:
                angle_plus = (message.data[0] << 24) | (message.data[1] << 16) | (message.data[2] << 8) | message.data[3]
                angle_minus = (message.data[4] << 24) | (message.data[5] << 16) | (message.data[6] << 8) | message.data[7]
                print(f"Received steering angle data: {angle_plus}, {angle_minus}")
            else:
                print("Invalid data length for steering torque message")
        elif message.arbitration_id == Constant.STM32_STEERING_STATES:
            data = ''.join([chr(byte) for byte in message.data]).rstrip('\x00')
            if data == "RUNNING":
                current_time = time.time()
                if self._last_running_message_time != 0:
                    time_diff = current_time - self._last_running_message_time
                    if time_diff > 1.5:  # Allow some tolerance
                        print(f"Steering control Warning: Message delayed by {time_diff:.2f} seconds!")
                self._last_running_message_time = current_time
            elif data == "ESTOP":
                print(f"Steering control is stopped!")
            else:
                print(f"Unexpected data for STM32_STEERING_STATES: {data}")

# Example usage
if __name__ == "__main__":
    can_connection = CANConnection(system="Windows", bitrate=500000)
    steering_control = SteeringControl(can_connection)

    # Keep the program running to receive messages
    try:
        while True:
            steering_control.set_torque(0)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Steering control stopped by user")
    finally:
        can_connection.close()

