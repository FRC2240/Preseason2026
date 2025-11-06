package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climber extends SubsystemBase{
    public TalonFX motorC = new TalonFX(67); //Change id (Climber) 

    public void setClimberPosition(AngleUnit rotations) {
        motorC.setPosition(Angle.ofBaseUnits(10, rotations));
    }

    public StatusSignal<Angle> getPosition(){
        return motorC.getPosition();
    } 
    
    public Command climberAngle(AngleUnit rotations) {
        return Commands.runOnce(() -> {
            setClimberPosition(rotations);
        }, this); 

    }
  }
