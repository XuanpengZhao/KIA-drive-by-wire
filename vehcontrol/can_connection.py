import usb.core
import can
import time
import os
from typing import Callable
from constant import Constant

class CANConnection:
    def __init__(self, system="Windows", bitrate=500000):
        self._system = system
        self._bitrate = bitrate
        self._bus = None
        self._notifier = None
        self._listeners = []
        if self._system == "Windows":
            self._bus = self._initialize_can_adapter_windows()
        elif self._system == "Linux":
            self._bus = self._initialize_can_adapter_linux()

    def _initialize_can_adapter_windows(self):
        try:
            # Find the USB device
            dev = usb.core.find(idVendor=Constant.GS_USB_ID_VENDOR, idProduct=Constant.GS_USB_ID_PRODUCT)
            if dev is None:
                raise ValueError("USB2CAN Device not found")

            # Initialize the CAN bus
            bus = can.Bus(interface='gs_usb', channel=dev.product, bus=dev.bus, address=dev.address, bitrate=self._bitrate)

            
            self._notifier = can.Notifier(bus, self._listeners)
            print(f"CAN adapter initialized: {bus}")
            return bus
        except can.CanError as e:
            print(f"Failed to initialize CAN adapter: {e}")
            return None
    
    def _initialize_can_adapter_linux(self):
        try:
            os.system("sudo chmod 666 can0") ######### Linux give access; may not work #########

            os.system(f"sudo ip link set can0 type can bitrate {self._bitrate}")
            os.system("sudo ip link set up can0")
            # Initialize the CAN bus
            bus = can.interface.Bus(channel='can0', bitrate=self._bitrate, bustype='socketcan')
            self._notifier = can.Notifier(self._bus, self._listeners)
            print(f"CAN adapter initialized: {bus}")
            return bus
        except can.CanError as e:
            print(f"Failed to initialize CAN adapter: {e}")
            return None

    def send_message(self, can_id, data):
        try:
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self._bus.send(message)
            # print(f"Message sent on {self._bus.channel_info}")
        except can.CanError as e:
            print(f"Message NOT sent: {e}")

    def add_listener(self, listener: Callable[[can.Message], None]):
        self._listeners.append(listener)
        if self._notifier:
            self._notifier.add_listener(listener)

    def close(self):
        if self._notifier:
            self._notifier.stop()
        if self._bus:
            self._bus.shutdown()
            if self._system == "Linux":
                os.system("sudo ip link set down can0")
            print("CAN adapter on can0 shut down")

if __name__ == "__main__":
    can_connection = CANConnection()
    def receive_message(message):
        # print(message)
        can_id = message.arbitration_id
        if can_id == Constant.STM32_STEERING_ANGLE:
            angle_plus = (message.data[0] << 24) | (message.data[1] << 16) | (message.data[2] << 8) | message.data[3]
            angle_minus = (message.data[4] << 24) | (message.data[5] << 16) | (message.data[6] << 8) | message.data[7]
            print(f"Received steering angle data: {angle_plus}, {angle_minus}")
        # print(f"Received message with ID {can_id:X}. Data: {data}")

    can_connection.add_listener(receive_message)
    try:
        while True:
            # Example ADC values
            adc1Value = 1550
            adc2Value = 1550

            # Prepare CAN message data
            data = [
                (adc1Value >> 24) & 0xFF, (adc1Value >> 16) & 0xFF, (adc1Value >> 8) & 0xFF, adc1Value & 0xFF,
                (adc2Value >> 24) & 0xFF, (adc2Value >> 16) & 0xFF, (adc2Value >> 8) & 0xFF, adc2Value & 0xFF
            ]

            can_connection.send_message(can_id=Constant.STM32_STEERING_TORQUE, data=data)
            time.sleep(1)

            # # Send emergency stop message
            # can_connection.send_message(arbitration_id=0x7D2, data=b'STOP')
            # time.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        can_connection.close()
