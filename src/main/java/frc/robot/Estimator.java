package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Estimator {
    private final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDrivePoseEstimator poseEstimator;

    public Estimator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(),
                drivetrain.getHeading(), drivetrain.getModulePositions(), new Pose2d(new Translation2d(0,0), new Rotation2d(0)));

        SmartDashboard.putData("Field", field);
    }

    public void update() {
        poseEstimator.update(drivetrain.getHeading(), drivetrain.getModulePositions());
        field.setRobotPose(getPose()); // Update advantageScope / logs
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }
}
