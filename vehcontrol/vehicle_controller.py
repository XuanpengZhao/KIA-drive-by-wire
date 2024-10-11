from can_connection import CANConnection
from steering_control import SteeringControl
from accelerator_control import AcceleratorControl
from brake_control import BrakeControl
import keyboard
from inputs import get_gamepad
import time

class VehicleController:
    def __init__(self):
        self.can_connection = CANConnection(system="Windows", bitrate=500000)
        self.steering_control = SteeringControl(self.can_connection)
        self.accelerator_control = AcceleratorControl(self.can_connection)
        self.brake_control = BrakeControl(self.can_connection)
        self._idle_brake = 0.1 # 0 ~ 1 value set to brake for countering the idle creep
        self._is_park = True
        self.set_brake_pedal_position(self._idle_brake)

    def set_steering_angle(self, angle):
        self.steering_control.set_torque(angle)

    def set_accelerator_position(self, position):
        if position > 0:
            self.set_brake_pedal_position(0)  # Lock brake position to 0 if accelerator is pressed
        self.accelerator_control.set_accelerator_position(position)

    def set_brake_pedal_position(self, position):
        if position > 0:
            self.set_accelerator_position(0)  # Lock accelerator position to 0 if brake is pressed
        self.brake_control.set_brake_position(position)

    def emergency_stop(self, stopOrReset):
        if stopOrReset == "STOP":
            self.steering_control.emergency_stop('STOP')
            self.brake_control.emergency_stop('STOP')
            self.accelerator_control.emergency_stop('STOP')
        elif stopOrReset == "RESET":
            self.steering_control.emergency_stop('RESET')
            self.brake_control.emergency_stop('RESET')
            self.accelerator_control.emergency_stop('RESET')

    def keyboard_control(self):
        if keyboard.is_pressed('up'):
            self.set_accelerator_position(0.5)
        elif keyboard.is_pressed('down'):
            self.set_brake_pedal_position(0.5)
        elif keyboard.is_pressed('p'): # park mode, counter idle creep
            self._is_park = True
        elif keyboard.is_pressed('l'):
            self._is_park = False
        else:
            self.set_accelerator_position(0)
            self.set_brake_pedal_position(0)

        if keyboard.is_pressed('left'):
            self.set_steering_angle(-0.5)
        elif keyboard.is_pressed('right'):
            self.set_steering_angle(0.5)
        else:
            self.set_steering_angle(0)

        if keyboard.is_pressed('s'):
            self.emergency_stop('STOP')
        elif keyboard.is_pressed('R'):
            self.emergency_stop('RESET')
        
        if self._is_park:
            self.set_brake_pedal_position(self._idle_brake)

        

    def gamepad_control(self):
         
        events = get_gamepad()
       
        for event in events:
            # print(event.code, ": ", event.state)
            if event.code == 'ABS_X': # Left joystick controls steering
                # print("ABS_X: ", event.state/32768)
                self.set_steering_angle(event.state / 32768)  # Normalize to -1 to 1
            elif event.code == 'ABS_RZ':
                # print("ABS_RZ: ", event.state/255) # Right trigger controls accelerator
                self.set_accelerator_position(event.state / 255)  # Normalize to 0 to 1
            elif event.code == 'ABS_Z':
                # print("ABS_Z: ", event.state/255) # Left trigger controls brake
                self.set_brake_pedal_position(event.state / 255)# Normalize to 0 to 1
            elif event.code == 'BTN_SOUTH' and event.state == 1:
                self._is_park = True # park mode, counter idle creep
            elif event.code == 'BTN_NORTH' and event.state == 1:
                self._is_park = False
            elif event.code == 'BTN_START' and event.state == 1:
                self.emergency_stop('STOP')
            elif event.code == 'BTN_SELECT' and event.state == 1:
                self.emergency_stop('RESET')

        if self._is_park:
            self.set_brake_pedal_position(self._idle_brake)
        
        

    def shutdown(self):# gives a minimum brake position to counter idle creep
        pass
        # self.can_connection.close()

# Example usage
if __name__ == "__main__":
    controller = VehicleController()
    
    try:
        while True:
            controller.gamepad_control()
            # time.sleep(0.1)
         
    finally:
        controller.shutdown()
