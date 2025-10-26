// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.vision.RealLimelightVisionIO;
import frc.robot.vision.SimPhotonVisionIO;
import frc.robot.vision.Vision;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Drivetrain drivetrain = new Drivetrain(joystick);
    public final Vision vision;

    public RobotContainer() {
        if (RobotBase.isReal()) {
            vision = new Vision(drivetrain::addVisionMeasurement,
                    new RealLimelightVisionIO("limelight-left", drivetrain::getHeading),
                    new RealLimelightVisionIO("limelight-right", drivetrain::getHeading));
        } else {
            vision = new Vision(drivetrain::addVisionMeasurement,
                    new SimPhotonVisionIO("camera_0", drivetrain::getPose, Constants.Vision.CAMERA_0_POS),
                    new SimPhotonVisionIO("camera_1", drivetrain::getPose, Constants.Vision.CAMERA_0_POS));
        }

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveCommand());
        joystick.b().toggleOnTrue(drivetrain.driveSlowCommand()); // Drives slow
        RobotModeTriggers.disabled().whileTrue(drivetrain.idleCommand()); // Idles the swerve on disable
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // Rezero Gyro
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
