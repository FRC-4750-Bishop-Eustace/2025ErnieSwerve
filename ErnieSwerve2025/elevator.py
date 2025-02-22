import math
from rev import SparkFlex, SparkFlexConfig, SparkBase
from wpimath.controller import ProfiledPIDController, ElevatorFeedforward
import wpimath.kinematics
import variables
import wpimath.geometry

kMaxVelocity = 50  # TBD - Currently an arbitrary value
kMaxAcceleration = 125  # TBD - Currently an arbitrary value

class Elevator:
    def __init__(
        self,
        elevatorMotor1ID: int,
        elevatorMotor2ID: int,
        heights: list[int]
    ) -> None:

        self.elevatorMotor1 = SparkFlex(elevatorMotor1ID, SparkFlex.MotorType.kBrushless)
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorConfig1 = SparkFlexConfig()
        self.elevatorMotor1.configure(self.elevatorConfig1, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.elevatorMotor2 = SparkFlex(elevatorMotor2ID, SparkFlex.MotorType.kBrushless)
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()
        self.elevatorConfig2 = SparkFlexConfig()
        self.elevatorMotor2.configure(self.elevatorConfig2, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.heights = heights

        self.elevatorPIDController = ProfiledPIDController(
            variables.elevatorPID_P,
            variables.elevatorPID_I,
            variables.elevatorPID_D,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kMaxVelocity,
                kMaxAcceleration
            )
        )

        self.elevatorFeedforward = ElevatorFeedforward(
            variables.elevatorFF_1,
            variables.elevatorFF_2,
            variables.elevatorFF_3,
            variables.elevatorFF_4
        )

    def start_elevatorMotor(self, setpoint: float) -> None:
        """Start the elevator motor with the given setpoint."""
        position1 = self.elevatorEncoder1.getPosition()
        position2 = self.elevatorEncoder2.getPosition()
        
        if position1 > 0 and position2 <= self.heights[3]:
            avg = (position1 - position2) / 2  # Adjusted to account for opposite directions
            encoderRotation = wpimath.geometry.Rotation2d.fromRotations(avg)

            output = self.elevatorPIDController.calculate(
                encoderRotation.degrees(), setpoint
            )

            feedforward = self.elevatorFeedforward.calculate(
                self.elevatorPIDController.getSetpoint().velocity
            )
            
            voltage = output + feedforward
            self.elevatorMotor1.setVoltage(voltage)
            self.elevatorMotor2.setVoltage(-voltage)

    def stop_elevatorMotor(self) -> None:
        """Stop the elevator motor."""
        self.elevatorMotor1.setVoltage(0)
        self.elevatorMotor2.setVoltage(0)
    
    def get_elevatorEncoder(self) -> None:
        """Print the elevator encoder positions and velocities."""
        print('encoder revolutions =', self.elevatorEncoder1.getPosition())
        print('encoder velocity =', self.elevatorEncoder2.getVelocity())

    def set_elevatorEncoder(self, x: float) -> None:
        """Set the elevator encoder positions."""
        print('Setting Encoder to:', x)
        self.elevatorEncoder1.setPosition(x)
        self.elevatorEncoder2.setPosition(-x)
    
    def set_elevatorMode(self, mode: int) -> None:
        """Set the elevator mode based on the given mode."""
        setpoints = {
            0: 0,
            1: self.heights[0],
            2: self.heights[1],
            3: self.heights[2],
            4: self.heights[3]
        }
        self.start_elevatorMotor(setpoints.get(mode, 0))
