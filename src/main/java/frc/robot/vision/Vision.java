package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.BaseVisionIO.vision_configuration_type;

import static frc.robot.Constants.Vision.ANGULAR_STDEV_COEFF;
import static frc.robot.Constants.Vision.ANGULAR_STDEV_MEGATAG_2_COEFF;
import static frc.robot.Constants.Vision.LINEAR_STDEV_COEFF;
import static frc.robot.Constants.Vision.LINEAR_STDEV_MEGATAG_2_COEFF;
import static frc.robot.Constants.Vision.APRIL_TAG_LAYOUT;
import static frc.robot.Constants.Vision.MAX_UNCERTAINTY;
import static frc.robot.Constants.Vision.MAX_Z_ERROR;

import java.util.LinkedList;
import java.util.List;
import java.util.zip.ZipException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.wpilibj.Alert; disconnected logging

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    // empty but can hold an object that implements vision consumer
    private final vision_consumer consumer;
    // empty array that can accept any object implementing the interface
    private final BaseVisionIO[] IO_base;
    // AutoLogged auto generated class
    private final BaseVisionIOInputAutoLogged[] input;

    // elipces means multiple objects of vision_IO_Base class can be passed in so
    // multiple cameras
    public Vision(vision_consumer consumer, BaseVisionIO... IO_base) {
        // this passes the private final consumer in
        // so running the method uses parameters to define private final variables which
        // then can't be changed

        this.consumer = consumer;
        this.IO_base = IO_base;

        this.input = new BaseVisionIOInputAutoLogged[IO_base.length];
        for (int i = 0; i < IO_base.length; i++) {
            input[i] = new BaseVisionIOInputAutoLogged();
        }

    }

    // returns X angle to nearest tag, method
    public Rotation2d getAngleX(int cameraIndex) {
        return input[cameraIndex].angle_to_tag.rot_x();
    }

    // updates input and logs for each camera
    @Override
    public void periodic() {
        for (int i = 0; i < IO_base.length; i++) {
            IO_base[i].update_inputs(input[i]);
            //Logger.processInputs("Vision/Camera" + i, input[i]);
        }

        // stores data for all cameras
        List<Pose3d> all_tag_poses = new LinkedList<>(); // position of all tags seen
        List<Pose3d> all_robot_poses = new LinkedList<>(); // all calculated robot poses
        List<Pose3d> all_accepted_poses = new LinkedList<>(); // all accepted (reasonable) positions
        List<Pose3d> all_rejected_poses = new LinkedList<>(); // all rejected (outside theshold unrealistic)

        // does a bunch of suff for each camera
        for (int i = 0; i < IO_base.length; i++) {
            List<Pose3d> tag_poses = new LinkedList<>();
            List<Pose3d> robot_poses = new LinkedList<>();
            List<Pose3d> accepted_poses = new LinkedList<>();
            List<Pose3d> rejected_poses = new LinkedList<>();

            // each tag seen ID appends its position to tag poses
            for (int tag_ID : input[i].april_tag_IDs) {
                var tag_pose = APRIL_TAG_LAYOUT.getTagPose(tag_ID);
                if (tag_pose.isPresent()) {
                    tag_poses.add(tag_pose.get());
                }
            }

            for (var estimation : input[i].pose_estimation_data) {
                // confirms estimation data is with in thresholds
                // example has many unecissary seeming conditions
                // come back to
                // potential point of failure

                boolean tagCountInvalid = estimation.april_tag_count() == 0;
                boolean uncertaintyInvalid = (estimation.april_tag_count() == 1 && estimation.uncertainty() > MAX_UNCERTAINTY);
                boolean zErrorInvalid=  Math.abs(estimation.position().getZ())  > MAX_Z_ERROR;
                boolean xOutBounds = (estimation.position().getX() < 0.0) &&  (estimation.position().getX() > APRIL_TAG_LAYOUT.getFieldLength());
                boolean yOutBounds = (estimation.position().getY() < 0.0) && (estimation.position().getY() > APRIL_TAG_LAYOUT.getFieldWidth());

                /*
                System.out.println("TagCount: "+tagCountInvalid);
                System.out.println("Uncertainty: " + uncertaintyInvalid);
                System.out.println("Zerror: " +zErrorInvalid);
                System.out.println("X Out of bounds: " + xOutBounds);
                System.out.println("Y out of bounds: " + yOutBounds);
                */

                boolean reject_pose =  tagCountInvalid || uncertaintyInvalid || zErrorInvalid || xOutBounds || yOutBounds;
                        

                

                robot_poses.add(estimation.position()); // stores all robot positions for a camera
                if (!reject_pose) {
                    accepted_poses.add(estimation.position()); // stores only accepted poses
                } else {
                    rejected_poses.add(estimation.position()); // stores onlt rejected poses
                }

                if (reject_pose) {
                    continue; // skips to next loop
                }

                // calculate stdev
                // stdev is an aproximation it is hueristic as the real thing can't be found
                //for some reason this baseline works with a multiplier to find x, y, and theta stdevs
                double stdev_factor = Math.pow(estimation.average_tag_distance(), 2.0)/estimation.april_tag_count();
                double linear_stdev = LINEAR_STDEV_COEFF * stdev_factor;
                double angular_stdev = ANGULAR_STDEV_COEFF * stdev_factor;

                //MTag2 configs
                if (estimation.type() == vision_configuration_type.METATAG_2) {
                    linear_stdev *=  LINEAR_STDEV_MEGATAG_2_COEFF;
                    angular_stdev *= ANGULAR_STDEV_MEGATAG_2_COEFF;
                }

                // TODO used if one camera is more trustworthy
                /* 
                if () {
                    
                }
                */

                //sends vision data
                
                consumer.accepts(
                    estimation.timestamp(),
                    estimation.position().toPose2d(),
                    // x, y, and theta
                    //x = y
                    VecBuilder.fill(linear_stdev, linear_stdev, angular_stdev));
            }

            //logs data by camera
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/Tag_positions",
                tag_poses.toArray(new Pose3d[tag_poses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/Robot_positions",
                robot_poses.toArray(new Pose3d[robot_poses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/Accepted_position",
                accepted_poses.toArray(new Pose3d[accepted_poses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/Rejected_positions",
                rejected_poses.toArray(new Pose3d[rejected_poses.size()]));

            // stores data for each camera
            all_tag_poses.addAll(tag_poses);
            all_robot_poses.addAll(robot_poses);
            all_accepted_poses.addAll(accepted_poses);
            all_rejected_poses.addAll(rejected_poses);

        }
    }

    // marks a function interface
    // functional interface allows for 1 method here accepts()
    @FunctionalInterface
    // consumer accepts but returns nothing
    public static interface vision_consumer {
        public void accepts(
                double timestamp,
                Pose2d robot_pose,
                // how much uncertainty there is
                // <N3, N1> takes 3 standerd devaitions from vector
                Matrix<N3, N1> stdevs);
    }
}