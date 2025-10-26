package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterCommands extends SubsystemBase implements ShooterSubsystemIO{
    ShooterSubsystem shooter = new ShooterSubsystem();

    public void override_setShooterAngle(AngleUnit rotations) {
        shooter.setShooterAngle(rotations);
    }

    public void override_setShooterOutput(Current current) {
        shooter.setShooterOutput(current);
    }

    public void override_shoot() {
        shooter.shoot();
    }

    // Indexer stuff down here

    public void override_intakeGP() {
        shooter.intakeGP();
    }

    public void override_extakeGP() {
        shooter.extakeGP();
    }

    public void override_indexGP() {
        shooter.indexGP();
    }
    
}
