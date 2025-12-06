// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick1 = new CommandXboxController(1);

    public final Drivetrain drivetrain = new Drivetrain(joystick);
    public final Vision vision = Vision.createVision(drivetrain);
    public final ObjectDetection objectDetection = new ObjectDetection("limelight", Constants.Vision.CAMERA_LL_POS ,drivetrain::getPose);
    public final Grabber grabber = new Grabber();
    public final Wrist wrist = new Wrist();
    public final Elevator elevator = new Elevator();


    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveCommand());
        joystick.b().toggleOnTrue(drivetrain.driveSlowCommand()); // Drives slow
        RobotModeTriggers.disabled().whileTrue(drivetrain.idleCommand()); // Idles the swerve on disable
        joystick.button(1).onTrue(drivetrain.zeroGyroWithAllianceCommand()); // Rezero Gyro with alliance

        joystick1.button(1).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.button(2).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.button(3).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.button(4).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
