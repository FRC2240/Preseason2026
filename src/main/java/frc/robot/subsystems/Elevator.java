package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class Elevator extends SubsystemBase {
    public TalonFX leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    public TalonFX rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/position", leftMotor.getPosition().getValueAsDouble());
    }

    public Angle getPosition() {
        return leftMotor.getPosition().getValue();
    }

    public void setPosition(Angle position) {
        MathUtils.clamp(position, Constants.Elevator.MIN_POSITION, Constants.Elevator.MAX_POSITION);
        leftMotor.setControl(req.withPosition(position));
    }

    public Command setPositionCommand(Angle position) {
        return Commands.run(() -> {
            setPosition(position);
        }, this).withName("Set Elevator Position").until(() -> {
            return getPosition().isNear(position, Constants.Elevator.POSITION_THRESHOLD);
        });
    }

    public Command elevatorOffsetsCommand(Angle amount) {
        return Commands.runOnce(() -> {
            setPosition(getPosition().plus(amount));
        });
    };
}
