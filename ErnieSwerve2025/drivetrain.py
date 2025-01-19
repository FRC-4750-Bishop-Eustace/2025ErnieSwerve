#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import navx
import pathplannerlib.path
import pathplannerlib.pathfinding
import pathplannerlib.trajectory
import wpilib
import wpimath.geometry
import wpimath.geometry
import wpimath.kinematics
import navx.src
import navx.src.rpy
import wpimath.trajectory
import swervemodule
import variables
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
import limelight.limelight as llutil
from navx import AHRS
from wpimath.estimator import SwerveDrive6PoseEstimator
import pathplannerlib.pathfinders as pathfinder

'''
kMaxSpeed = 1.5  # meters per second
kRMaxSpeed = 0.1
kTMaxSpeed = 1.0
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = wpimath.geometry.Rotation2d(0.3)
'''

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        # NOTE: May need to tweak the center measurements 44.5mm rough measurement
        # NOTE: EVERYTHING IS MEASURE IN METERS! 
        # NOTE: Update center measure distance from each module
        # Wheel to Wheel = 63.4 cm
        # Chassis = 76cm
        '''
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.32, 0.32)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.32, -0.32)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.32, -0.32)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.32, 0.32)
        '''
        self.frontLeftLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, variables.chassisHalfLength)
        self.frontRightLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backRightLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backLeftLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, variables.chassisHalfLength)

        self.frontLeft = swervemodule.SwerveModule(variables.frontLeftDriveController, variables.frontLeftTurnController, variables.frontLeftDriveEncoder, variables.frontLeftTurnEncoder)
        self.frontRight = swervemodule.SwerveModule(variables.frontRightDriveController, variables.frontRightTurnController, variables.frontRightDriveEncoder, variables.frontRightTurnEncoder)
        self.backRight = swervemodule.SwerveModule(variables.backRightDriveController, variables.backRightTurnController, variables.backRightDriveEncoder, variables.backRightTurnEncoder)
        self.backLeft = swervemodule.SwerveModule(variables.backLeftDriveController, variables.backLeftTurnController, variables.backLeftDriveEncoder, variables.backLeftTurnEncoder)

        '''
        BERT NOTES:
        
        driveMotorID: int,
        turningMotorID: int,
        driveEncoderID: int,
        turningEncoderID: int,

        Front Left (+,+) 
            - Drive Motor: 4
            - Rotation Motor: 3
            - Drive Encoder: 4
            - Rotation Encoder: 13 
        
        Front Right (+,-) 
            - Drive Motor: 7
            - Rotation Motor: 8
            - Drive Encoder: 7
            - Rotation Encoder: 10

        Rear Left (-,+) 
            - Drive Motor: 2
            - Rotation Motor: 1
            - Drive Encoder: 2
            - Rotation Encoder: 11

        Rear Right (-,-) 
            - Drive Motor: 5
            - Rotation Motor: 6
            - Drive Encoder: 5
            - Rotation Encoder: 12

        '''

        self.gyro = wpilib.AnalogGyro(0)
        self.angler = navx.AHRS.create_spi()
        print("gyroscope = ", self.angler)
        self.gyroinit = self.angler.getAngle()
        self.gyroradiansinit = wpimath.units.degreesToRadians(self.gyroinit)
        print("gyro", self.gyro)

        #NOTE: Just defining the fixed kinematics of the bot
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backRightLocation,
            self.backLeftLocation,
        )

        self.initialPose = wpimath.geometry.Pose2d()

        #NOTE: getPosition - need to determine position value - velocity and angle -
        #NOTE: Need to understand expected units/values returned - is it meters & radians?
        self.estimator = SwerveDrive6PoseEstimator(self.kinematics, self.gyro.getRotation2d(), [self.frontLeft.getPosition(), self.frontRight.getPosition(), self.backLeft.getPosition(), self.backRight.getPosition], self.initialPose)

        self.angler.reset()
    
    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        #if fieldRelative:
        #    self.updateOdometry()

        self.gyro = self.angler.getAngle()
        self.gyroradians = wpimath.units.degreesToRadians(self.gyro)

        #print("gyro", self.gyroradians)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, wpimath.geometry.Rotation2d(self.gyroradians)
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, variables.kMaxSpeed
        )
        
        #NOTE: Should we desaturate for Turning speed motors? 
        
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backRight.setDesiredState(swerveModuleStates[2])
        self.backLeft.setDesiredState(swerveModuleStates[3])

        print(swerveModuleStates[0])

    def stop(self) -> None:
        self.drive(0, 0, 0, False, 0.02)

    # CURRENLY NOT BEING USED
    def updateOdometry(self, useAprilTags: bool) -> None:
        """Updates the field relative position of the robot."""
        self.estimator.update(self.gyro.getRotation2d(), [self.frontLeft.getPosition(), self.frontRight.getPosition(), self.backLeft.getPosition(), self.backRight.getPosition])

        # Find discoverable Limelight cameras
        if useAprilTags and llutil.searchForLimelights(debug=True):
            llutil.setRobotOrientation("limelight", self.getRotation().degrees(), 0, 0, 0, 0, 0)
            mt2 = llutil.getRobotPoseEstimateBlueMT2("limelight")

            if not math.abs(self.gyro.getRate()) > 720 or not mt2.tagCount == 0:
                self.estimator.setVisionMeasurementStdDevs([0.7, 0.7, 9999999])
                self.estimator.addVisionMeasurement(mt2.pose, mt2.timestamp)
    
    def alignment(self) -> None:
        self.frontLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.frontRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.backRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.backLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))

    def autoTest(self) -> None:
        self.frontLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))
        self.frontRight.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))
        self.backRight.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))
        self.backLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(1, wpimath.geometry.Rotation2d(1)))

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.estimator.getEstimatedPosition()
    
    def getPosition(self) -> wpimath.geometry.Translation2d:
        return self.getPose().translation()
    
    def getRotation(self) -> wpimath.geometry.Rotation2d:
        return self.getPose().rotation()

    def travelTo(self, pose: wpimath.geometry.Pose2d, timeout: float = 20) -> None:
        start = wpilib.Timer.getFPGATimestamp()

        while True:
            current = self.getPose()
            positionError = pose.translation().distance(current.translation())
            rotationError = pose.rotation().degrees() - current.rotation().degrees()

            # Normalize rotation error to the range [-180, 180]
            rotationError = (rotationError + 180) % 360 - 180

            # Check if the target pose has been reached or if timeout has occurred
            if positionError < 0.1 and abs(rotationError) < 2 or (wpilib.Timer.getFPGATimestamp() - start) > timeout:
                break

            # Calculate speeds (proportional control)
            xSpeed = (pose.X() - current.X()) * 0.5  # Proportional gain for X
            ySpeed = (pose.Y() - current.Y()) * 0.5  # Proportional gain for Y
            rotSpeed = rotationError * 0.01  # Proportional gain for rotation

            # Clamp speeds to maximum allowed values
            xSpeed = max(min(xSpeed, variables.kMaxSpeed), -variables.kMaxSpeed)
            ySpeed = max(min(ySpeed, variables.kMaxSpeed), -variables.kMaxSpeed)
            rotSpeed = max(min(rotSpeed, variables.kMaxAngularSpeed), -variables.kMaxAngularSpeed)

            self.drive(xSpeed, ySpeed, rotSpeed, True, 0.02)

        self.stop() # Stop robot

    def followPath(self, path: wpimath.geometry.Pose2d, timeout: float = 20) -> None:
        for point in path:
            self.travelToPoint(point, timeout)
        self.stop() # Stop robot