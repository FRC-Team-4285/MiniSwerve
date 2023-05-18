package org.usfirst.frc4285.CamoSwerve.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4285.CamoSwerve.Robot;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

public class FieldCentricSwerveDrive extends Command {

    public static final double OMEGA_SCALE = 1.0 / 36.0;
    public static final double DEADZONE = 0.05;

    private double originHeading = 0.0;
    private double originCorrection = 0;
    
    public FieldCentricSwerveDrive() {
        requires(Robot.drive);
    }

    @Override
    protected void initialize() {
        originHeading = Robot.zeroHeading;
    }

    @Override
    protected void execute() {
        if (!Robot.oi.getLeftJoyButton(7) | !Robot.oi.getRightJoyButton(7)) {
			originHeading = RobotMap.navX.getFusedHeading();
		}

        // One Joystick
        /*
        double strafe = Robot.oi.rightJoy.getX() * -1;
        double forward = Robot.oi.rightJoy.getY();
        double omega = Robot.oi.rightJoy.getX() * OMEGA_SCALE * -1;
        */

        /* One Joystick
        double strafe = Robot.oi.rightJoy.getX();
        /double forward = Robot.oi.rightJoy.getY() * -1;
        double omega = Robot.oi.rightJoy.getZ() * OMEGA_SCALE;
        */

        // One Joystick - different brand
        // double strafe = Robot.oi.controller.getRawAxis(0);
        // double forward = Robot.oi.controller.getRawAxis(1) * -1;
        // double omega = Robot.oi.controller.getRawAxis(2) * OMEGA_SCALE;

	// Xbox Controller
        double strafe = Robot.oi.controller.getRawAxis(0);
        double forward = Robot.oi.controller.getRawAxis(1) * -1;
        double omega = Robot.oi.controller.getRawAxis(4) * OMEGA_SCALE;

        // Two Joysticks
        // double strafe = Robot.oi.leftJoy.getX();
        // double forward = Robot.oi.leftJoy.getY() * -1;
        // double omega = Robot.oi.leftJoy.getZ() * OMEGA_SCALE; // was rightJoy.getX()

        // Add a small deadzone on the joysticks
        if (Math.abs(strafe) < DEADZONE) {
			strafe = 0.0;
		}

        if (Math.abs(forward) < DEADZONE) {
			forward = 0.0;
		}

        if (Math.abs(omega) < DEADZONE * OMEGA_SCALE) {
			omega = 0.0;
		}
    
        // If all of the joysticks are in the deadzone, don't update the motors
        // This makes side-to-side strafing much smoother
        if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
            Robot.drive.setDriveLeftFront(0.0);
            Robot.drive.setDriveLeftRear(0.0);
            Robot.drive.setDriveRightFront(0.0);
            Robot.drive.setDriveRightRear(0.0);
            return;
        }

        if (!Robot.oi.leftJoy.getTrigger()) {
            // When the Left Joystick trigger is not pressed, The robot is in Field Centric Mode.
            // The calculations correct the forward and strafe values for field centric attitude. 

            // Rotate the velocity vector from the joystick by the difference between our
            // current orientation and the current origin heading
            double originCorrection = Math.toRadians(originHeading - RobotMap.navX.getFusedHeading());
            double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
            strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
            forward = temp;
        }

        Robot.drive.swerveDrive(strafe, forward, omega);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }
}
