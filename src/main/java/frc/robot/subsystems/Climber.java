package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public TalonFX climber = new TalonFX(Constants.Climber.CLIMBER_MOTOR_ID);
    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public Climber() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimit = 3;

        climber.getConfigurator().apply(cfg);
    }

    public void setPosition(Angle rotations) {
        climber.setControl(req.withPosition(rotations));
    }

    public Angle getPosition() {
        return climber.getPosition().getValue();
    }

    public Command setPositionCommand(Angle rotations) {
        return Commands.run(() -> {
            setPosition(rotations);
        }, this).until(() -> {
            return getPosition().isNear(rotations, Constants.Wrist.POSITION_THRESHOLD);
        });
    }
}
