package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    public TalonFX motorI = new TalonFX(100); //Change id (Intaking)
    public TalonFX motorS = new TalonFX(101); //Change id (Shooting)
    MotionMagicVelocityTorqueCurrentFOC req = new MotionMagicVelocityTorqueCurrentFOC(0);

    public Indexer() {
        TalonFXConfiguration index = new TalonFXConfiguration();
        index.CurrentLimits.SupplyCurrentLimit = 3;
    }
    
    public void setIndexerSpeed(Double Speed) {
        motorI.setControl(req.withVelocity(Speed)); //Units is rotations/s
    }

    public void Shoot() {
        motorS.setControl(req.withVelocity(5)); //Units is rotations/s
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
