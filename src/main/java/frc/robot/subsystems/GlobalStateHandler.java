package frc.robot.subsystems;

import frc.robot.Constants;

public class GlobalStateHandler extends SubsystemBase{
    Constants.ManipulatorStates currentState;


    public Command setStateCommand(Constants.ManipulatorStates target) {
        Constants.ManipulatorStates lastState = 
        currentState = target;

        if(target)
    }
}
