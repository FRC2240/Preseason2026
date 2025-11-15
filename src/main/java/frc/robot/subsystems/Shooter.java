package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    // Use different motor id for your own subsystem by making it inside constants
    public TalonFX pivot = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    public TalonFX shooter = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public Shooter() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimit = 3;

        pivot.getConfigurator().apply(cfg);

    }

    public void setShooterAngle(AngleUnit rotations) {
        pivot.setControl(req.withPosition(Angle.ofBaseUnits(10, rotations)));
    }

    // spd is -1.0 to 1.0
    public void setShooterOutput(double spd) {
        shooter.set(spd);
    }

    public Command shooterAngleCommand(AngleUnit rotations) {
        return Commands.runOnce(() -> {
            setShooterAngle(rotations);
        }, this);
    }

    // spd is -1.0 to 1.0
    public Command shooterOutputCommand(double spd) {
        return Commands.run(() -> {
            setShooterOutput(spd);
        }, this);
    }
}
