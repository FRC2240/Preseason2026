package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    public TalonFX motorA = new TalonFX(39);
    public TalonFX motorR = new TalonFX(38);

    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public ShooterSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimit = 3;

    }

    public void setShooterAngle(AngleUnit rotations) {
        motorA.setControl(req.withPosition(Angle.ofBaseUnits(10, rotations)));
    }

    // spd is -1.0 to 1.0
    public void setShooterOutput(double spd) {
        motorR.set(spd);
    }

    public void shoot() {
        // Remove if the idex is what pushes note in so it gets shot
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

    public Command shootCommand() {
        return Commands.runOnce(() -> {
            shoot();
        }, this);
    }
}
