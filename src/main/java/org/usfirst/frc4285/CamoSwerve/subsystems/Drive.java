/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import java.util.Arrays;
import java.util.Collections;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc4285.CamoSwerve.RobotMap;
import org.usfirst.frc4285.CamoSwerve.commands.FieldCentricSwerveDrive;
import org.usfirst.frc4285.CamoSwerve.commands.SimpleSwerveDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Drive extends Subsystem {
    /*
     * Drive Subsystem
     *
     * This class implements a swerve drive terrain. Code is designed
     * for use with our 2020 robot. Dimensions will need to be updated
     * if this code is desired for use in your own robot.
     */

    public CANSparkMax driveLeftFrontSpark;
    public CANSparkMax driveLeftRearSpark;
    public CANSparkMax driveRightFrontSpark;
    public CANSparkMax driveRightRearSpark;

    private SparkMaxPIDController driveLeftFrontController;
    private SparkMaxPIDController driveLeftRearController;
    private SparkMaxPIDController driveRightFrontController;
    private SparkMaxPIDController driveRightRearController;

    private TalonSRX steerLeftFront;
    private TalonSRX steerLeftRear;
    private TalonSRX steerRightFront;
    private TalonSRX steerRightRear;

    // Wheel base lengths are measured from where wheels touch the ground
    public static final double WHEEL_BASE_LENGTH = 11.25; //measured from center of each module
    public static final double WHEEL_BASE_WIDTH = 11.25;

    // Encoder counts are 1024 for ma3. 4096 for ctre mag encoders
    public static final double ENCODER_COUNT_PER_ROTATION = 1024.0; 

    public static final double WHEEL_DIAMETER = 4.0;

    // Max speed is 0 to 1 
    public static final double MAX_SPEED = 0.3; //was 0.3

    // Apply a governor to the speed output. Use 1.0 to disable.
    public static final double SPEED_GOVERNOR = 3.0; //was 0.5
    public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;

    // Drive inches per count is calculated for cimcoders under motor with a final gear reduction of 6.67
    // which is the reduction for the andymark swerve and steer.
    public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
    public static final double DEADZONE = 0.11;
    public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

    // The steering PIDs need to be adjusted for your drive. Start with I = D = 0.
    // Set P low and try moving. If no oscilation, double P. When steer oscillates,
    // set your P to the last value that did not oscillate. Set D to about P * 10 to start.
    private static final double DRIVE_P = 10.0; //was 7.5
    private static final double DRIVE_I = 0.1; //was 0.02
    private static final double DRIVE_D = 10.0; //was 50

    private static final double STEER_P = 10.0; //was 7
    private static final double STEER_I = 0.1; //was 0.02
    private static final double STEER_D = 5.0; //was 40
    private static final int STATUS_FRAME_PERIOD = 5;

    public Drive() {
        ////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // Drive Motors - These motors determine the speed the robot drives.
        //
        ////////////////////////////////////////////////////////////////////////////////////////////////

        // Left Front Drive Motor Module
        driveLeftFrontSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
        driveLeftFrontSpark.restoreFactoryDefaults();
        driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
        driveLeftFrontSpark.setInverted(true); //was false
        driveLeftFrontSpark.setOpenLoopRampRate(0.125);
        driveLeftFrontSpark.setSmartCurrentLimit(60);
		driveLeftFrontController = driveLeftFrontSpark.getPIDController();
		driveLeftFrontController.setP(DRIVE_P);
		driveLeftFrontController.setI(DRIVE_I);
		driveLeftFrontController.setD(DRIVE_D);
		driveLeftFrontController.setIZone(0);

        // Left Rear Drive Motor Module
        driveLeftRearSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_REAR_ID, MotorType.kBrushless);
        driveLeftRearSpark.restoreFactoryDefaults();
        driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
        driveLeftRearSpark.setInverted(true);//was false
        driveLeftRearSpark.setOpenLoopRampRate(0.125);
        driveLeftRearSpark.setSmartCurrentLimit(60);
		driveLeftRearController = driveLeftRearSpark.getPIDController();
		driveLeftRearController.setP(DRIVE_P);
		driveLeftRearController.setI(DRIVE_I);
		driveLeftRearController.setD(DRIVE_D);
		driveLeftRearController.setIZone(0);

        // Right Front Drive Motor Module
        driveRightFrontSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
        driveRightFrontSpark.restoreFactoryDefaults();
        driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
        driveRightFrontSpark.setInverted(true); //was false
        driveRightFrontSpark.setOpenLoopRampRate(0.125);
        driveRightFrontSpark.setSmartCurrentLimit(60);
		driveRightFrontController = driveRightFrontSpark.getPIDController();
		driveRightFrontController.setP(DRIVE_P);
		driveRightFrontController.setI(DRIVE_I);
		driveRightFrontController.setD(DRIVE_D);
		driveRightFrontController.setIZone(0);

        // Right Rear Drive Motor Module
        driveRightRearSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_REAR_ID, MotorType.kBrushless);
        driveRightRearSpark.restoreFactoryDefaults();
        driveRightRearSpark.setIdleMode(IdleMode.kBrake);
        driveRightRearSpark.setInverted(false); //was true
        driveRightRearSpark.setOpenLoopRampRate(0.125);
        driveRightRearSpark.setSmartCurrentLimit(60);
		driveRightRearController = driveRightRearSpark.getPIDController();
		driveRightRearController.setP(DRIVE_P);
		driveRightRearController.setI(DRIVE_I);
		driveRightRearController.setD(DRIVE_D);
		driveRightRearController.setIZone(0);

        ////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // Steer Motors - These motors determine the direction the robot drives in.
        //
        ////////////////////////////////////////////////////////////////////////////////////////////////

        // Left Front Steering Motor Module
        steerLeftFront = new TalonSRX(RobotMap.STEER_LEFT_FRONT_ID);
        steerLeftFront.configFactoryDefault();
        steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftFront.configFeedbackNotContinuous(false, 10);
        steerLeftFront.setSensorPhase(true);
        steerLeftFront.setInverted(false);
        steerLeftFront.config_kP(0, STEER_P, 0);
        steerLeftFront.config_kI(0, STEER_I, 0);
        steerLeftFront.config_kD(0, STEER_D, 0);
        steerLeftFront.config_IntegralZone(0, 100, 0);
        steerLeftFront.configAllowableClosedloopError(0, 5, 0);
        steerLeftFront.setNeutralMode(NeutralMode.Brake);
        steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        // Left Rear Steering Motor Module
        steerLeftRear = new TalonSRX(RobotMap.STEER_LEFT_REAR_ID);
        steerLeftRear.configFactoryDefault();
        steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftRear.configFeedbackNotContinuous(false, 10);
        steerLeftRear.setSensorPhase(true);
        steerLeftRear.setInverted(false); //was true
        steerLeftRear.config_kP(0, STEER_P, 0);
        steerLeftRear.config_kI(0, STEER_I, 0);
        steerLeftRear.config_kD(0, STEER_D, 0);
        steerLeftRear.config_IntegralZone(0, 100, 0);
        steerLeftRear.configAllowableClosedloopError(0, 5, 0);
        steerLeftRear.setNeutralMode(NeutralMode.Brake);
        steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        // Right Front Steering Motor Module
        steerRightFront = new TalonSRX(RobotMap.STEER_RIGHT_FRONT_ID);
        steerRightFront.configFactoryDefault();
        steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightFront.configFeedbackNotContinuous(false, 10);
        steerRightFront.setSensorPhase(true);
        steerRightFront.setInverted(false); //was true
        steerRightFront.config_kP(0, STEER_P, 0);
        steerRightFront.config_kI(0, STEER_I, 0);
        steerRightFront.config_kD(0, STEER_D, 0);
        steerRightFront.config_IntegralZone(0, 100, 0);
        steerRightFront.configAllowableClosedloopError(0, 5, 0);
        steerRightFront.setNeutralMode(NeutralMode.Brake);
        steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        // Right Rear Steering Motor Module
        steerRightRear = new TalonSRX(RobotMap.STEER_RIGHT_REAR_ID);
        steerRightRear.configFactoryDefault();
        steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightRear.configFeedbackNotContinuous(false, 10);
        steerRightRear.setSensorPhase(true);
        steerRightRear.setInverted(false);
        steerRightRear.config_kP(0, STEER_P, 0);
        steerRightRear.config_kI(0, STEER_I, 0);
        steerRightRear.config_kD(0, STEER_D, 0);
        steerRightRear.config_IntegralZone(0, 100, 0);
        steerRightRear.configAllowableClosedloopError(0, 5, 0);
        steerRightRear.setNeutralMode(NeutralMode.Brake);
        steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    }
  
    public void swerveDrive(double strafe, double forward, double omega) {
        /*
         * Main method for calculating motor output for swerve drive terrain.
         * based on driver input on their controls.
         */

        double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
        double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);

        // Compute the constants used later for calculating speeds and angles
        double A = strafe - omegaL2;
        double B = strafe + omegaL2;
        double C = forward - omegaW2;
        double D = forward + omegaW2;

        // Compute the drive motor speeds
        double speedLF = speed(B, D);
        double speedLR = speed(A, D);
        double speedRF = speed(B, C);
        double speedRR = speed(A, C);

        // Angles for the steering motors.

        // If drives are calibrated for zero position on encoders they are at 90 degrees
        // to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
        // for initial position/calibration of drives.

        // NOTE: This is only needed if the drives are calibrated to zero facing the sides
        // instead of front back.

        double angleLF = 0.0;
        double angleLR = 0.0;
        double angleRF = 0.0;
        double angleRR = 0.0;

        // 1 - Practice Bot
        // 2 - Main Robot
        // 3 - Practice Bot (Alt Config)
        int robot_config = 1;
        if (robot_config == 1) {
            // 2022 Main Robot Configuration - Fixed Wheels

            // Mini Swerve Bot Configuration
            angleLF = angle(B, D) + 5.0; //was 70
            angleLR = angle(A, D) + 100.0; //was -10
            angleRF = angle(B, C) - 25.0; //was 170
            angleRR = angle(A, C) + 4.0; //was 10

           //2022 Main Bot Angles
           // angleLF = angle(B, D) - 90; // A1
           // angleLR = angle(A, D) - 163; // A4
           // angleRF = angle(B, C) - 88; // A3
           // angleRR = angle(A, C) - 88; // A2
        }

        // Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
        double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

        // Set each swerve module, scaling the drive speeds by the maximum speed
        setSwerveModule(steerLeftFront, driveLeftFrontSpark, angleLF, speedLF / maxSpeed);
        setSwerveModule(steerLeftRear, driveLeftRearSpark, angleLR, speedLR / maxSpeed);
        setSwerveModule(steerRightFront, driveRightFrontSpark, angleRF, speedRF / maxSpeed);
        setSwerveModule(steerRightRear, driveRightRearSpark, angleRR, speedRR / maxSpeed);
    }

    private double speed(double a, double b) {
        /*
         * Calculate drive motor speed.
         */

        return Math.hypot(a, b) * SPEED_GOVERNOR;
    }
  
    private double angle(double a, double b) {
        /*
         * Calculate steering motor angle.
         */

        return Math.toDegrees(Math.atan2(a, b));
    }

    private void setSwerveModule(TalonSRX steer, CANSparkMax drive, double angle, double speed) {
        /*
         * Sets an individual swerve module's speed and angle. This method handles the background
         * calculations required to applying desired angles and speed. Simply provide the spark
         * encoder for the motor you wish to update, the new desired angle, and the new desired speed. 
         */

        double currentPosition = steer.getSelectedSensorPosition(0);
        double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;

        // The angle from the encoder is in the range [0, 360], but the swerve computations
        // return angles in the range [-180, 180], so transform the encoder angle to this range
        if (currentAngle > 180.0) {
            currentAngle -= 360.0;
        }

        // todo: Properly invert the steering motors so this isn't necessary
        // This is because the steering encoders are inverted
        double targetAngle = -angle;
        double deltaDegrees = targetAngle - currentAngle;

        // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
        if (Math.abs(deltaDegrees) > 180.0) {
            deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
        }

        // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
        // only rotate by the complement, decreasing the amount of time required to rotate the wheel.
        if (Math.abs(deltaDegrees) > 90.0) {
            // Apply adjustment new angle.
            deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
            // Wheel is now facing the opposite the direction it was previously,
            // therefore we need to negate its speed to continue driving in the
            // proper direction.
            speed = -speed;
        }

        // Determine final position for our steering motor.
        double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;

        // Set the steering motor position.
        steer.set(ControlMode.Position, targetPosition);

        // Drive the motor.
        drive.set(speed);
    }

    public double getSteerLFEncoder() {
        /*
         * Returns the encoder value for Left Front steer motor.
         */

        return steerLeftFront.getSelectedSensorPosition(0);
    }

    public double getSteerLREncoder() {
        /*
         * Returns the encoder value for Left Rear steer motor.
         */

        return steerLeftRear.getSelectedSensorPosition(0);
    }

    public double getSteerRFEncoder() {
        /*
         * Returns the encoder value for Right Front steer motor.
         */

        return steerRightFront.getSelectedSensorPosition(0);
    }
    
    public double getSteerRREncoder() {
        /*
         * Returns the encoder value for Right Rear steer motor.
         */

        return steerRightRear.getSelectedSensorPosition(0);
    }

    public void setDriveLeftFront(double speed) {
        /*
         * Sets motor speed for Left Front drive motor.
         */

        driveLeftFrontSpark.set(speed);
    }

    public void setDriveLeftRear(double speed) {
        /*
         * Sets motor speed for Left Rear drive motor.
         */

        driveLeftRearSpark.set(speed);
    }

    public void setDriveRightFront(double speed) {
        /*
         * Sets motor speed for Right Front drive motor.
         */

        driveRightFrontSpark.set(speed);
    }

    public void setDriveRightRear(double speed) {
        /*
         * Sets motor speed for Right Rear drive motor.
         */

        driveRightRearSpark.set(speed);
    }

    @Override
    public void initDefaultCommand() {
        /*
         * Sets FieldCentricSwerveDrive as the default command
         * for this subsystem.  
         */
        boolean want_simple_drive = false;

        if (want_simple_drive) {
            setDefaultCommand(new SimpleSwerveDrive());
        }
        else {
            setDefaultCommand(new FieldCentricSwerveDrive());
        }
    }
}
