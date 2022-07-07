/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class RobotMap {
    public static AHRS navX; 

    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int CONTROLLER = 2;

    public static final int DRIVE_LEFT_FRONT_ID = 3;
    public static final int DRIVE_LEFT_REAR_ID = 4;
    public static final int DRIVE_RIGHT_FRONT_ID = 2;
    public static final int DRIVE_RIGHT_REAR_ID = 1;

    public static final int STEER_LEFT_FRONT_ID = 7; // 5
    public static final int STEER_LEFT_REAR_ID = 8;  // 6  
    public static final int STEER_RIGHT_FRONT_ID = 6; // 8
    public static final int STEER_RIGHT_REAR_ID = 5; // 7

    // Use Phoenix Diagnostics Tool + USB to roborio to find this.

    //IDs for the climber's arms. Right they share IDs with other parts of the bot.. for some reason.
    //public static final int LEFT_ARM_ID = 11;
    //public static final int RIGHT_ARM_ID = 10;
    //public static final int BALL_INTAKE_ID = ;

    public static void init() {
        navX = new AHRS(SPI.Port.kMXP);
        navX.setAngleAdjustment(90);
    }
}
