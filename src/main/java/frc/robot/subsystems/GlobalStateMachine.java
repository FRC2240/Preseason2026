package frc.robot.subsystems;

import frc.robot.Constants.ManipulatorStates;

public class GlobalStateMachine{
    Elevator elevator;
    Grabber grabber;
    Wrist wrist;

    ManipulatorStates currentState = ManipulatorStates.IDLE;

    public GlobalStateMachine(Elevator elevatorI, Grabber grabberI, Wrist wristI) {
        this.elevator = elevatorI;
        this.grabber = grabberI;
        this.wrist = wristI;
    }

    public void setStateCommand(ManipulatorStates target, boolean hasGP) {
        ManipulatorStates lastState = currentState;
        currentState = target;

        //intakes
        if(currentState == ManipulatorStates.ALGAE_INTAKE) {
            // grabber currently has no algae specific commands
        }
        if(currentState == ManipulatorStates.INTAKE){
            this.grabber.intakeCommand();
        }

        //positions
        if( currentState == ManipulatorStates.IDLE){
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
        }


        //extakes
        if(currentState == ManipulatorStates.ALGAE_EXTAKE) {
            // grabber currently has no algae specific commands
        }
        if(currentState == ManipulatorStates.EXTAKE) {
            this.grabber.ejectCommand();
        }
    }
    
}
