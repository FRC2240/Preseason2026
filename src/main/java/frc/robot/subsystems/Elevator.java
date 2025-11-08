package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class Elevator extends SubsystemBase{
    public TalonFX left_motor = new TalonFX(Constants.Elevator.MOTOR_ID);
    public TalonFX right_motor = new TalonFX(Constants.Elevator.MOTOR_ID_FOLLOW);

    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);
    
    public Elevator(){
        MotionMagicConfigs mconf = new MotionMagicConfigs();
        TalonFXConfiguration conf = new TalonFXConfiguration();
        mconf.MotionMagicAcceleration = 250;
        mconf.MotionMagicCruiseVelocity = 35;
    
        conf.Slot0.kP = 40;
        conf.Slot0.kS = 4.5;
        conf.Slot0.kD = 10;

        conf.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        left_motor.getConfigurator().apply(conf);
        right_motor.getConfigurator().apply(conf);

        right_motor.setControl(new Follower(Constants.Elevator.MOTOR_ID, true));
    }

    public Angle getPosition(){
        return left_motor.getPosition().getValue();
    }

    public Command setPositionCommand(Angle Position) {
        return Commands.runOnce(() -> {
            left_motor.setControl(req.withPosition(Position));
        }, this).withName("89ufw").until(() -> {
            return getPosition().isNear(Position, Constants.Elevator.POSITION_THRESHOLD);
        });
    }


}
