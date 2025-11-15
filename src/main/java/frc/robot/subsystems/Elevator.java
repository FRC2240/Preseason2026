package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class Elevator extends SubsystemBase {
    public TalonFX leftMotor = new TalonFX(Constants.Elevator.MOTOR_ID);
    public TalonFX rightMotor = new TalonFX(Constants.Elevator.MOTOR_ID_FOLLOW);

    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public Elevator() {
        TalonFXConfiguration conf = new TalonFXConfiguration();
        conf.MotionMagic.MotionMagicAcceleration = 250;
        conf.MotionMagic.MotionMagicCruiseVelocity = 35;

        conf.Slot0.kP = 40;
        conf.Slot0.kS = 4.5;
        conf.Slot0.kD = 10;

        conf.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        leftMotor.getConfigurator().apply(conf);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    public Angle getPosition() {
        return leftMotor.getPosition().getValue();
    }

    public Command setPositionCommand(Angle Position) {
        return Commands.runOnce(() -> {
            leftMotor.setControl(req.withPosition(Position));
        }, this).withName("Set Elevator Position").until(() -> {
            return getPosition().isNear(Position, Constants.Elevator.POSITION_THRESHOLD);
        });
    }

    public Command elevatorOffsetsCommand(Angle amount) {
        return Commands.runOnce(() -> {
            leftMotor.setControl(req.withPosition(getPosition().plus(amount)));
        });
    };
}
