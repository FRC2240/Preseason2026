package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorStates;

public class GlobalStateMachine extends SubsystemBase{
    Elevator elevator;
    Grabber grabber;
    Wrist wrist;

    ManipulatorStates currentState = ManipulatorStates.IDLE;

    public GlobalStateMachine(Elevator elevatorI, Grabber grabberI, Wrist wristI) {
        this.elevator = elevatorI;
        this.grabber = grabberI;
        this.wrist = wristI;
    }

    public Command setStateCommand(ManipulatorStates target, boolean hasGP) {
        ManipulatorStates lastState = currentState;
        currentState = target;

        //intakes
        if(currentState == ManipulatorStates.ALGAE_INTAKE) {
            // Do Algae Intake
        }
        if(currentState == ManipulatorStates.INTAKE){
            // Do Coral Intake
        }

        //positions
        if(currentState == ManipulatorStates.IDLE){
            // Set pos Idle
        }

        if(currentState == ManipulatorStates.L1) {
            // Set pos L1
        }

        if(currentState == ManipulatorStates.L2) {
            // Set pos L2
        }

        if(currentState == ManipulatorStates.L3) {
            // Set pos L3
        }

        if(currentState == ManipulatorStates.L4) {
            // Set pos L4
        }

        if(currentState == ManipulatorStates.ALGAE_GROUND) {
            // Set pos Algae Ground
        }

        if(currentState == ManipulatorStates.ALGAE_L2) {
            // Set pos Algae L2
        }

        if(currentState == ManipulatorStates.ALGAE_L3) {
            // Set pos Algae L3
        }

        if(currentState == ManipulatorStates.BARGE) {
            // Set pos Barge
        }

        if(currentState == ManipulatorStates.PROCESSOR) {
            // Set pos Processor
        }


        //extakes
        if(lastState != ManipulatorStates.IDLE) {
            if(currentState == ManipulatorStates.ALGAE_EXTAKE) {
                
            }
            if(currentState == ManipulatorStates.EXTAKE) {
                // Do Coral extake
            }
        }
    }
    
}
