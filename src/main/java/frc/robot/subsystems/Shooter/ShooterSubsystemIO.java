package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Current;

public interface ShooterSubsystemIO {

    void override_setShooterAngle(AngleUnit rotations);

    void override_setShooterOutput(Current current);

    void override_shoot();

    // Indexer stuff down here

    void override_intakeGP();

    void override_extakeGP();

    void override_indexGP();
}
