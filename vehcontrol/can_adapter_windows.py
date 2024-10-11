import time
from gs_usb.gs_usb import GsUsb
from gs_usb.constants import GS_CAN_MODE_NORMAL

class CANAdapterWindows:
    def __init__(self, bitrate=500000):
        self.bitrate = bitrate
        self.dev = None

    def initialize(self):
        if self.dev:
            # Set bitrate
            if not self.dev.set_bitrate(self.bitrate):
                print("Failed to set bitrate")
                return False

            # Start the device in normal mode
            self.dev.start(GS_CAN_MODE_NORMAL)

            print("CAN adapter initialized and ready for data")
        return True

    def get_device_info(self):
        devs = GsUsb.scan()
        if len(devs) == 0:
            print("No gs_usb device found")
            return None
        
        # Select the first device
        self.dev = devs[0]
        print(f"Found device: {self.dev}")
        return None


    def close(self):
        if self.dev:
            self.dev.stop()
            self.dev = None

# Usage example
if __name__ == "__main__":
    adapter = CANAdapterWindows()
    device_info = adapter.get_device_info()
    