// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Drivetrain drivetrain = new Drivetrain(joystick);
    public final Vision vision = Vision.createVision(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveCommand());
        joystick.b().toggleOnTrue(drivetrain.driveSlowCommand()); // Drives slow
        RobotModeTriggers.disabled().whileTrue(drivetrain.idleCommand()); // Idles the swerve on disable
        joystick.button(1).onTrue(drivetrain.zeroGyroWithAllianceCommand()); // Rezero Gyro with alliance
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
