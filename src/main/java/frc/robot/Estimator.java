package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Estimator {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDrivePoseEstimator poseEstimator;

    public Estimator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(),
                drivetrain.getHeading(), drivetrain.getModulePositions(), null);
    }

    public void update() {
        poseEstimator.update(drivetrain.getHeading(), drivetrain.getModulePositions());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

}
