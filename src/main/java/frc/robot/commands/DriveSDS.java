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
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    controller.getLeftY() * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                    controller.getLeftX() * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                    controller.getRightX() * DrivetrainSDS.MAX_VELOCITY_METERS_PER_SECOND,
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}