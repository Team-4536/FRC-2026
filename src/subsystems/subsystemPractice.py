# FIX: Use 'subsystems.filename' so Python finds them in the subfolder
from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.desiredState import DesiredState
from ntcore import NetworkTableInstance
from wpimath.controller import PIDController 

class SubsystemExample(Subsystem):

    def __init__(self):
        # Setup telemetry for dashboard feedback
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        
        # Initialize the motor with the correct device ID
        self.subsystemMotor = RevMotor(deviceID=8)
        
        # PID Controller (Smart Speed Control)
        # kP = 0.005: Adjust this if the motor is too slow or too aggressive
        self.pid = PIDController(0.005, 0, 0) 

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState) -> None:
        # Send target value to NetworkTables for debugging
        self.table.putNumber("revMotorTarget", ds.revMotor)

        # Check the Deadband: Only run if joystick is pushed more than 10%
        if abs(ds.revMotor) > 0.1:
            # 1. Get the actual RPM from the motor
            actual_speed = self.subsystemMotor.getVelocity()
            
            # 2. Set our speed goal (120 RPM)
            target_rpm = 120.0
            
            # 3. Calculate necessary power using PID
            power = self.pid.calculate(actual_speed, target_rpm)
            
            # 4. Limit power between -1.0 and 1.0 (Safety clamp)
            power = max(min(power, 1.0), -1.0)
            
            self.subsystemMotor.set(power)
        else:
            # If the stick is released, stop immediately
            self.disabled()

    def disabled(self) -> None:
        # Reset power and PID memory to prevent 'jumping' when restarted
        self.subsystemMotor.set(0)
        self.pid.reset() 

    def publish(self):
        # Show actual speed on the dashboard to verify the PID
        self.table.putNumber("actualVelocity", self.subsystemMotor.getVelocity())

