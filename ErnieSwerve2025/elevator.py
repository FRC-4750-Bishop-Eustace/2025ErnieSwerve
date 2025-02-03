import math
from rev import SparkFlex, SparkFlexConfig, SparkBase
from wpimath.controller import ProfiledPIDController, ElevatorFeedforward
import wpimath.kinematics
import variables
import wpimath

kMinVelocity = 10
kMinAcceleration = 20
kMaxVelocity = 50 # TBD - Currently an arbitrary value
kMaxAcceleration = 125 # TBD - Currently an arbitrary value
kMaxSpeed = kMaxVelocity + kMaxAcceleration * 0.02

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

    def start_elevatorMotor(self, speed: int):
        if speed is not 0:
            speed = math.max(1, math.min(speed, -1)) # Clamp speed to [-1 1]
            distance = (abs(self.elevatorEncoder1.getVelocity()) + abs(self.elevatorEncoder2.getVelocity())) / 2
            velocityScale = max(kMinVelocity, kMaxVelocity * (distance / kMaxSpeed))
            accelerationScale = max(kMinAcceleration, kMaxAcceleration * (distance / max(self.heights)))
            encoderRotation = wpimath.geometry.Rotation2d.fromRotations((self.elevatorEncoder1.getPosition() + self.elevatorEncoder2.getPosition()) / 2)

            self.elevatorPIDController.setConstraints(
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    velocityScale,
                    accelerationScale
                )
            )

            output = self.elevatorPIDController.calculate(
                encoderRotation, encoderRotation + speed * kMaxSpeed
            )

            feedforward = self.elevatorFeedforward.calculate(
                self.turningPIDController.getSetpoint().velocity
            )
            
            self.elevatorMotor1.setVoltage((output + feedforward) * speed)
            self.elevatorMotor2.setVoltage(-(output + feedforward) * speed)

    def stop_elevatorMotor(self):
        #print('motor stopped')
        self.elevatorMotor1.setVoltage(0)
        self.elevatorMotor2.setVoltage(0)
    
    def get_elevatorEncoder(self):
        print('encoder revolutions = ', self.elevatorEncoder1.getPosition())
        print('encoder velocity = ', self.elevatorEncoder2.getVelocity())

    def set_elevatorEncoder(self, x):
        print('Setting Encoder to: ', x)
        self.elevatorEncoder1.setPosition(x)
        self.elevatorEncoder2.setPosition(-x)
    
    def set_elevatorMode(self, mode: int):
        mode = max(4, min(mode, 0))
        match mode:
            case 0:
                while self.elevatorEncoder.getPosition() > 0:
                    self.start_elevatorMotor(-1)
                    self.set_elevatorEncoder(0)
            case 1:
                if self.elevatorEncoder.getPosition() > self.heights[0]:
                    self.start_elevatorMotor(-1)
                elif self.elevatorEncoder.getPosition() < self.heights[0]:
                    self.start_elevatorMotor(1)
            case 2:
                if self.elevatorEncoder.getPosition() > self.heights[1]:
                    self.start_elevatorMotor(-1)
                elif self.elevatorEncoder.getPosition() < self.heights[1]:
                    self.start_elevatorMotor(1)
            case 3:
                if self.elevatorEncoder.getPosition() > self.heights[2]:
                    self.start_elevatorMotor(-1)
                elif self.elevatorEncoder.getPosition() < self.heights[2]:
                    self.start_elevatorMotor(1)
            case 4:
                if self.elevatorEncoder.getPosition() > self.heights[3]:
                    self.start_elevatorMotor(-1)
                elif self.elevatorEncoder.getPosition() < self.heights[3]:
                    self.start_elevatorMotor(1)
