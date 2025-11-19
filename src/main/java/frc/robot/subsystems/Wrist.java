package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Wrist extends SubsystemBase {
    private TalonFX motor = new TalonFX(Constants.Wrist.WRIST_MOTOR_ID);
    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public Wrist() {
        motor.setPosition(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotionMagic.MotionMagicAcceleration = 350;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 35;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        cfg.Slot0.kP = 24;
        cfg.Slot0.kD = 1;

        motor.getConfigurator().apply(cfg);

    }

    public void setPosition(Angle rotations) {
        motor.setControl(req.withPosition(rotations));
    }

    public Angle getPosition() {
        return motor.getPosition().getValue();
    }

    public Command setPositionCommand(Angle rotations) {
        return Commands.run(() -> {
            setPosition(rotations);
        }, this).until(() -> {
            return getPosition().isNear(rotations, Constants.Wrist.POSITION_THRESHOLD);
        });
    }

    public Command offsetCommand(Angle offset) {
        return Commands.runOnce(() -> {
            setPosition(getPosition().plus(offset));
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist/position", motor.getPosition().getValueAsDouble());
    }
}