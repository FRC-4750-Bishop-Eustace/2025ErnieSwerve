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
import drivetrain
import variables
import elevator
import navxGyro
import ntcore
#import limelight
#import limelightresults
import json
import time
#import limelight
#import vision
#import camera
#import auto
import ntcore
from cscore import CameraServer
from wpilib import SmartDashboard


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(variables.joystickPort1)
        self.controller2 = wpilib.Joystick(variables.joystickPort2)
        self.swerve = drivetrain.Drivetrain()

        self.elevator = elevator.Elevator(15)
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

        '''
        discovered_limelights = limelight.discover_limelights(debug=True)
        print("discovered limelights:", discovered_limelights)
        if discovered_limelights:
            limelight_address = discovered_limelights[0] 
            ll = limelight.Limelight(limelight_address)
            results = ll.get_results()
            status = ll.get_status()
            print("-----")
            print("targeting results:", results)
            print("-----")
            print("status:", status)
            print("-----")
            print("temp:", ll.get_temp())
            print("-----")
            print("name:", ll.get_name())
            print("-----")
            print("fps:", ll.get_fps())
            print("-----")
            print("hwreport:", ll.hw_report())

        ll.enable_websocket()
    
        # print the current pipeline settings
        print(ll.get_pipeline_atindex(0))

        # update the current pipeline and flush to disk
        pipeline_update = {
        'area_max': 98.7,
        'area_min': 1.98778
        }
        ll.update_pipeline(json.dumps(pipeline_update),flush=1)

        print(ll.get_pipeline_atindex(0))

        # switch to pipeline 1
        ll.pipeline_switch(1)

        # update custom user data
        ll.update_python_inputs([4.2,0.1,9.87])
    '''

    #FUTURE
    def autonomousInit(self):
        self.autoSelected = self.chooser.getSelected()
        print("Auto selected:" + self.autoSelected)

    def autonomousPeriodic(self) -> None:
        self.timer.start()
        match self.autoSelected:
            case self.customAuto:
                # custom auto code
                #self.timer.start()
                if 0.5 < self.timer.get() < 1.5:
                    self.stopAuto()
                if 1.5 < self.timer.get() < 2.5:
                    self.shooter.vacummotor()
                if 2.5 < self.timer.get() < 4.0:
                    self.shooter.stopIntakemotor()
                    self.slowAutoDrive()
                if self.timer.get() > 4.0:
                    self.shooter.stopShootermotor()
                    self.stopAuto()
                else:
                    self.shooter.speakershootmotor()
                    self.slowAutoDrive()
                '''
                if 1.5 < self.timer.get() < 3.0:
                    self.stopAuto()
                    self.shooter.speakershootmotor()
                if 3.0 < self.timer.get() < 4.0:
                    self.shooter.vacummotor()
                if 4.0 < self.timer.get() < 5.0:
                    self.shooter.stopIntakemotor()
                    self.shooter.stopShootermotor()
                    self.getAuto()
                if self.timer.get() > 6.0:
                    self.stopAuto()
                else:
                    self.slowAutoDrive()
                '''
            case _:
                # default auto code
                self.timer.start()
                if 2.0 < self.timer.get():
                    self.stopAuto()
                else:
                    self.slowAutoDrive()

        '''
        self.timer.start()
        #self.driveWithJoystick(False)
        self.swerve.updateOdometry()
        #self.swerve.autoTest()
        if 2.0 < self.timer.get():
            self.stopAuto()
        else:
            self.getAuto()
        
        if 1.5 > self.timer.get() > 1.0:
            self.stopAuto()
        if 2.0 > self.timer.get() > 1.5:
            self.strafeLeft()
        if 2.0 < self.timer.get():
            self.stopAuto()
        else:
            self.getAuto()
        '''

    def teleopPeriodic(self) -> None:

        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.tid = self.lmtable.getNumber('tid', None)
        self.hw = self.lmtable.getNumber('hw', None)
        self.botpose = self.lmtable.getEntry('botpose').getDoubleArray([])

        print('pose =', self.botpose)
        #print("hw", self.hw)

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

        if self.fieldDrive == 2:
            self.driveWithJoystick(True)
        else:
            self.driveWithJoystick(False)
    
        self.navxGyro.getGyro()
        
        if self.controller.getRawButton(variables.squareButton) == 1:
            self.swerve.alignment()
        
        if self.controller.getRawButton(variables.triangleButton) == 1:
            self.elevator.start_elevatorMotor()
        else:
            self.elevator.stop_elevatorMotor()


    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        if fieldRelative:
            self.swerve.updateOdometry()

        xSpeed = (
            self.xspeedLimiter.calculate(
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
                -wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
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
        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
    

        
