package org.usfirst.frc4285.CamoSwerve.commands;

import org.usfirst.frc4285.CamoSwerve.Robot;
import edu.wpi.first.wpilibj.command.Command;


public class SimpleSwerveDrive extends Command {

    public SimpleSwerveDrive() {
        requires(Robot.drive);
    }

    @Override
    protected void execute() {
        double vX = Robot.oi.leftJoy.getX();
        double vY = -Robot.oi.leftJoy.getY();
        double omega = Robot.oi.rightJoy.getX() / 30.0;
        Robot.drive.swerveDrive(vX, vY, omega);
    }

    @Override
      protected boolean isFinished() {
        return false;
    }

    @Override
      protected void end() {
    }

    @Override
      protected void interrupted() {
        end();
    }
}
