package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class RealPhotonVisionIO extends ParentPhotonVisionIO{
    public RealPhotonVisionIO(String name, Transform3d camera_pos){
        super(name, camera_pos);
    }

    @Override
    public void update_inputs(BaseVisionIOInput inputs) {
        //updates inputs done through parent class
        super.update_inputs(inputs);

        //accounts for latency does nothing if no results
        if (inputs.pose_estimation_data != null) {
            
        }
    }
}
