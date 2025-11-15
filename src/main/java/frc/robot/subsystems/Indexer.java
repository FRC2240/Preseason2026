package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public TalonFX indexer = new TalonFX(100); // Change id (Intaking)
    public TalonFX loader = new TalonFX(101); // Change id (Shooting)
    MotionMagicVelocityTorqueCurrentFOC req = new MotionMagicVelocityTorqueCurrentFOC(0);

    public Indexer() {
        TalonFXConfiguration indexCfg = new TalonFXConfiguration();
        indexCfg.CurrentLimits.SupplyCurrentLimit = 3;

        TalonFXConfiguration loadCfg = new TalonFXConfiguration();
        loadCfg.CurrentLimits.SupplyCurrentLimit = 3;

        indexer.getConfigurator().apply(indexCfg);
        loader.getConfigurator().apply(loadCfg);
    }

    public void setIndexerSpeed(Double Speed) {
        indexer.setControl(req.withVelocity(Speed)); // Units is rotations/s
    }

    public void Shoot() {
        loader.setControl(req.withVelocity(5)); // Units is rotations/s
    }

    public Command indexerSpeedCommand(Double Speed) {
        return Commands.runOnce(() -> {
            setIndexerSpeed(Speed);
        }, this);
    }

    public Command indexerShootCommand() {
        return Commands.runOnce(() -> {
            Shoot();
        }, this);
    }

}
