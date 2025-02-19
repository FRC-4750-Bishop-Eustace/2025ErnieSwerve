#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import wpimath.estimator
import drivetrain
import variables
import elevator
import elevator2
import navxGyro
import ntcore
#import limelight
#import limelightresults
import LimelightHelpers
import json
import time
#import limelight
#import vision
#import camera
#import auto
import ntcore
from wpilib import SmartDashboard, Field2d
from cscore import CameraServer
from wpilib import SmartDashboard
import choreo
# import urcl

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(variables.joystickPort1)
        self.controller2 = wpilib.Joystick(variables.joystickPort2)
        self.pad = wpilib.Joystick(variables.joystickPort3)

        self.swerve = drivetrain.Drivetrain()
        #self.odometry = 

        self.elevator = elevator.Elevator(16, 17, [100, 200, 300, 400])
        self.elevator2 = elevator2.Elevator(18)
        #self.limelight = limelight.PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, avgTagDist, avgTagArea, fiducials)

        # navxGyro is a file to test the navx Gyro. This can be ignored/commented out.
        self.navxGyro = navxGyro.Gyro()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # Speed limiters

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(variables.x_slewrate)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(variables.y_slewrate)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(variables.rot_slewrate)

        self.timer = wpilib.Timer()
        self.fieldDrive = 1
        #CameraServer.startAutomaticCapture()
        
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.lmtable = self.inst.getTable("limelight")

        self.field = Field2d()
        SmartDashboard.putData("field", self.field)
        #SmartDashboard.putData("Swerve", self.swerve)
        
        # Loads from deploy/choreo/myTrajectory.traj
        # ValueError is thrown if the file does not exist or is invalid
        
        try:
            self.trajectory = choreo.load_swerve_trajectory("Test15") # 
        except ValueError:
        # If the trajectory is not found, ChoreoLib already prints to DriverStation
            pass

        wpilib.DataLogManager.start()
        #urcl.start()

        #urcl.start(wpilib.DataLogManager.getLog())
        

    def robotPeriodic(self):
        self.swerve.updateOdometry()

    #FUTURE
    def autonomousInit(self):
        self.swerve.resetGyro()

        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance())
            
            #print(initial_pose)

            self.swerve.resetRobotPose(initial_pose)

            if initial_pose:
                # Reset odometry to the start of the trajectory
                self.swerve.updateOdometry()

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

        #self.autoSelected = self.chooser.getSelected()
        #print("Auto selected:" + self.autoSelected)

    def autonomousPeriodic(self) -> None:

        self.field.setRobotPose(self.swerve.odometry.getPose())

        if self.trajectory:
            # Sample the trajectory at the current time into the autonomous period
            sample = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance())

            if sample:
                if sample.timestamp > 11.5:
                    self.swerve.drive(0,0,0,True,self.getPeriod())
                else:
                    self.swerve.follow_trajectory(sample)
                    
                

    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def teleopInit(self) -> None:
        self.swerve.resetRobotPose(self.swerve.odometry.getPose())
        self.initPose = self.swerve.odometry.getPose()


    def teleopPeriodic(self) -> None:

        #self.swerve.updateOdometry()
        #self.field.setRobotPose(self.swerve.odometry.getPose())

        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.tid = self.lmtable.getNumber('tid', None)
        self.hw = self.lmtable.getNumber('hw', None)

        self.botpose = self.lmtable.getEntry('botpose_wpiblue').getDoubleArray([])

        #self.tagpose = self.botpose.toPose2d()
        #self.field.setRobotPose(self.botpose[0], self.botpose[1], self.botpose[2])

        self.swerve.UpdateEstimator()
        #print(self.visionPose)
        #self.swerve.estimator.addVisionMeasurement(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]), self.getPeriod())
        self.visionPose = self.swerve.estimator.getEstimatedPosition()
        #self.field.setRobotPose(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]))
        self.field.setRobotPose(self.visionPose)
        #print(self.visionPose)
        #print(self.swerve.odometry.getPose())
        #print(self.swerve.estimator.getEstimatedPosition())
        '''
        result = ll.get_latest_results()
        parsed_result = limelightresults.parse_results(result)
        if parsed_result is not None:
            print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
            #for tag in parsed_result.fiducialResults:
            #    print(tag.robot_pose_target_space, tag.fiducial_id)
        time.sleep(1)  # Set this to 0 for max fps
        '''

        # CHANGE TO FIELD DRIVE VS BOT RELETIVE
        if self.controller.getRawButton(variables.crossButton) == 1:
            self.fieldDrive = 2
        if self.controller.getRawButton(variables.circleButton) == 1:
            self.fieldDrive = 1

        if self.controller.getRawButton(variables.triangleButton) == 1:
            self.swerve.resetGyro()
        
        if self.controller.getRawButton(variables.squareButton) == 1:
            self.elevator2.start_elevatorMotor()
        else:
            self.elevator2.stop_elevatorMotor()

        if self.fieldDrive == 2:
            self.driveWithJoystick(True)
        else:
            self.driveWithJoystick(False)

        #print(self.controller.getPOV())
    
        self.navxGyro.getGyro()
        
        '''
        if self.pad.getRawButton(6) == 1:
            self.elevator.start_elevatorMotor(5)
        elif self.pad.getRawButton(9) == 1:
            self.elevator.start_elevatorMotor(-5)
        else:
            self.elevator.stop_elevatorMotor()
        '''
        

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        #if fieldRelative:
        #    self.swerve.updateOdometry()

        self.dPad = self.controller.getPOV()

        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), variables.x_deadband)
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), variables.y_deadband)
            )
            * variables.kTMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.

        '''
        rot = (
            (-self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed) +
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)
        )
        '''
        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                -wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)

        '''
        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(2), variables.rot_deadband)
            ) / 2)
            * variables.kRMaxSpeed
        )
        '''
        

        if self.dPad == 0:
            xSpeed = 0.2
        if self.dPad == 90:
            ySpeed = -0.2
        
        if self.dPad == 180:
            xSpeed = -0.2
        if self.dPad == 270:
            ySpeed = 0.2
        
        if self.controller.getRawButton(variables.L1Button) == 1:
            rot = 0.5
        if self.controller.getRawButton(variables.R1Button) == 1:
            rot = -0.5
    
        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def microAdjustmentMode(self, fieldRelative: bool) -> None:
        
        '''
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), variables.x_deadband)
            )
            * variables.kMaxSpeed
        )

        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), variables.y_deadband)
            )
            * variables.kTMaxSpeed
        )

        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                -wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)
        '''

        
        if self.dPad == -1:
            xSpeed = 0
            ySpeed = 0
            rot = 0
        

        variables.setTurnState(rot)

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
