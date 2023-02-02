package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSDS;

import java.util.function.DoubleSupplier;

public class DriveSDS extends CommandBase {
    private final DrivetrainSDS m_drivetrainSubsystem;

    private final XboxController controller;
    public DriveSDS(DrivetrainSDS drivetrainSubsystem, XboxController controller) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.controller = controller;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                // ChassisSpeeds.fromFieldRelativeSpeeds(
                //     ((Math.abs(controller.getLeftY()) > 0.1) ? controller.getLeftY() : 0) * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                //     ((Math.abs(controller.getLeftX()) > 0.1) ? controller.getLeftX() : 0) * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                //     ((Math.abs(controller.getRightX()) > 0.1) ? controller.getRightX() : 0) * DrivetrainSDS.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                //     m_drivetrainSubsystem.getGyroscopeRotation()
                // )
                new ChassisSpeeds(
                    ((Math.abs(controller.getLeftY()) > 0.1) ? controller.getLeftY() : 0) * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                    ((Math.abs(controller.getLeftX()) > 0.1) ? controller.getLeftX() : 0) * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                    ((Math.abs(controller.getRightX()) > 0.1) ? controller.getRightX() : 0) * DrivetrainSDS.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}