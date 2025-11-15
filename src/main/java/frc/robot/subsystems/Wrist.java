package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Wrist extends SubsystemBase{
    private final TalonFX wrist = new TalonFX(88);
    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public Wrist() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimit = 3;


        wrist.getConfigurator().apply(cfg);
    }

    public void setPosition(AngleUnit rotations) {
         wrist.setControl(req.withPosition(Angle.ofBaseUnits(10, rotations)));
    }

    public Command setPositionCommand(AngleUnit rotations) {
        return Commands.run(() -> {
            setPosition(rotations);
        }, this);
    } 
}