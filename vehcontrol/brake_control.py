import time
from can_connection import CANConnection
from constant import Constant

class BrakeControl:
    def __init__(self, can_connection):
        self._can_connection = can_connection
        self._BRAKE_MIN = 248
        self._BRAKE_MAX = 1272
        self._BRAKE_RANGE = self._BRAKE_MAX - self._BRAKE_MIN
        self._last_running_message_time = 0

        # Register the callback for receiving messages
        self._can_connection.add_listener(self.receive_message)

    def set_brake_position(self, position_value):
        # Convert position value from 0 to 1 range to actual CAN values
        position_1 = int(self._BRAKE_MIN + position_value * self._BRAKE_RANGE)
        position_2 = position_1 * 2

        data = [
            (position_1 >> 24) & 0xFF, (position_1 >> 16) & 0xFF, (position_1 >> 8) & 0xFF, position_1 & 0xFF,
            (position_2 >> 24) & 0xFF, (position_2 >> 16) & 0xFF, (position_2 >> 8) & 0xFF, position_2 & 0xFF
        ]
        print("Brake position sent: ", position_1, ", ", position_2)
        self._can_connection.send_message(Constant.STM32_BRAKE_POSITION, data)
    
    def emergency_stop(self, stopOrReset):
        if stopOrReset == "STOP":
            self._can_connection.send_message(Constant.STM32_BRAKE_ESTOP, b'STOP')
        elif stopOrReset == "RESET":
            self._can_connection.send_message(Constant.STM32_BRAKE_ESTOP, b'RESET')

    def receive_message(self, message):
        if message.arbitration_id == Constant.STM32_BRAKE_STATES:
            data = ''.join([chr(byte) for byte in message.data]).rstrip('\x00')
            if data == "RUNNING":
                current_time = time.time()
                if self._last_running_message_time != 0:
                    time_diff = current_time - self._last_running_message_time
                    if time_diff > 1.5:  # Allow some tolerance
                        print(f"Brake control Warning: Message delayed by {time_diff:.2f} seconds!")
                self._last_running_message_time = current_time
            elif data == "ESTOP":
                print(f"Brake control is stopped!")
            else:
                print(f"Unexpected data for STM32_BRAKE_STATES: {data}")

# Example usage
if __name__ == "__main__":
    can_connection = CANConnection(system="Windows", bitrate=500000)
    brake_control = BrakeControl(can_connection)

    # Keep the program running to receive messages
    try:
        while True:
            brake_control.set_brake_position(0.5)  # Example: Set brake to 50%
            time.sleep(1)
    except KeyboardInterrupt:
        print("Brake control stopped by user")
    finally:
        can_connection.close()
