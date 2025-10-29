package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AllianceRelativePose {
    private Pose2d bluePose;
    private Pose2d redPose;

    private AllianceRelativePose(Pose2d bluePose) {
        this.bluePose = bluePose;
        this.redPose = flipPose(bluePose);
    }

    public static Pose2d flipPose(Pose2d original) {
        double flippedX = Constants.Field.FIELD_LENGTH - original.getX();
        double flippedY = Constants.Field.FIELD_WIDTH - original.getY();
        Rotation2d flippedRot = original.getRotation().plus(Rotation2d.fromDegrees(180));

        return new Pose2d(new Translation2d(flippedX, flippedY), flippedRot);
    }

    public static AllianceRelativePose fromBluePose(Pose2d bluePose) {
        return new AllianceRelativePose(bluePose);
    }

    public static AllianceRelativePose fromRedPose(Pose2d redPose) {
        return new AllianceRelativePose(flipPose(redPose));
    }

    public Pose2d get() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty() || alliance.get() == Alliance.Red) {
            return redPose;
        }

        return bluePose;
    }
}
