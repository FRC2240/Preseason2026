package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Estimator {
    private final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDrivePoseEstimator poseEstimator;

    double[] stateStdDevs = { 0.05, 0.05, 0.005 }; // x, y, theta (rad)
    double[] visionStdDevs = { 0.5, 0.5, Double.MAX_VALUE }; // huge theta variance so vision never changes heading

    public Estimator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        Matrix<N3, N1> stateStdDevsMatrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), stateStdDevs);
        Matrix<N3, N1> vistionStdDevsMatrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), visionStdDevs);

        this.poseEstimator = new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(),
                drivetrain.getHeading(),
                drivetrain.getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                stateStdDevsMatrix,
                vistionStdDevsMatrix
        );

        // Creates smartdashboard widget
        SmartDashboard.putData("Field", field);
    }

    public void update() {
        poseEstimator.update(drivetrain.getHeading(), drivetrain.getModulePositions());
        field.setRobotPose(getPose()); // Update advantageScope / logs
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        // Makes sure that vision cannot give false heading estimates
        Pose2d correctedPose = new Pose2d(pose.getTranslation(), drivetrain.getHeading());
        poseEstimator.addVisionMeasurement(correctedPose, timestamp, stdDevs);
    }
}
