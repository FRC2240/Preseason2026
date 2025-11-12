package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.playingwithfusion.TimeOfFlight;


public class Grabber extends SubsystemBase{
    public TalonFX motorV = new TalonFX(Constants.Grabber.MOTOR_ID);
    public TimeOfFlight sensor = new TimeOfFlight(Constants.Grabber.SENSOR_ID);
    public CoastOut coast = new CoastOut();

    public Grabber() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 10;
        conf.Slot1.kS = 6;
        conf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        conf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorV.getConfigurator().apply(conf);
    }
    public boolean gamePiece() {
        Distance distance = Millimeter.of(sensor.getRange());
        return distance.compareTo(Constants.Grabber.INTAKE_Distance) <= 0;
    }      

    public TorqueCurrentFOC req = new TorqueCurrentFOC(0);  

    public void setVelocity(Current current) {
        motorV.setControl(req.withOutput(current));
    }

    public double getRange() {
        return sensor.getRange();
    }

    public Command coastCommand() {
        return Commands.runOnce(() -> {
            motorV.setControl(coast);
        }, this);
    }

    public Command intakeCommand() {
        return Commands.runOnce(() -> {
            setVelocity(Constants.Grabber.INTAKE_VELOCITY);
        }, this);
    }

   public Command ejectCommand() {
        return Commands.runOnce(() -> {
        setVelocity(Constants.Grabber.EJECT_VELOCITY);
        }, this);
    }

    public Command idleCommand() {
        return Commands.runOnce(() -> {
        setVelocity(Amps.of(0));
        }, this);
    }


}


