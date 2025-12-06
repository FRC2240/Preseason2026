package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase {
    final int entriesPerObject = 12;

    private final Supplier<Pose2d> poseSupplier;
    private final DoubleArraySubscriber rawDetectionSubscriber;
    private final Transform3d cameraPose;
    private final Field2d field = new Field2d();
    
    private Pose2d latestAlgae = null;
    private Pose2d latestCoral = null;

    // Ty to meters
    private final InterpolatingDoubleTreeMap tyDistanceMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(-7.5, 0.3048), // 1.0 ft
            Map.entry(2.5, 0.4572), // 1.5 ft
            Map.entry(8.2, 0.6096), // 2.0 ft
            Map.entry(10.7, 0.762), // 2.5 ft
            Map.entry(14.7, 0.9144), // 3.0 ft
            Map.entry(16.3, 1.0668), // 3.5 ft
            Map.entry(18.0, 1.2192), // 4.0 ft
            Map.entry(19.9, 1.3716), // 4.5 ft
            Map.entry(20.6, 1.524), // 5.0 ft
            Map.entry(21.8, 1.6764), // 5.5 ft
            Map.entry(22.5, 1.8288), // 6.0 ft
            Map.entry(23.1, 1.9812), // 6.5 ft
            Map.entry(23.6, 2.1336) // 7.0 ft
    );

    public ObjectDetection(String name, Transform3d cameraPose, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.cameraPose = cameraPose;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        rawDetectionSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(null);
        SmartDashboard.putData("ObjectDetection", field);
    }

    public Pose2d getLatestAlgae() {
        return latestAlgae;
    }

    public Pose2d getLatestCoral() {
        return latestCoral;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = poseSupplier.get();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double[] rawDetections = rawDetectionSubscriber.get();
        if (rawDetections == null)
            return;
        int objectCount = rawDetections.length / entriesPerObject;
        List<Pose2d> detectedObjects = new ArrayList<>();

        for (int i = 0; i < objectCount; i++) {
            int baseIndex = i * entriesPerObject;
            int classId = (int) rawDetections[baseIndex];
            double tx = rawDetections[baseIndex + 1];
            double ty = rawDetections[baseIndex + 2];
            double camY = cameraPose.getTranslation().getY();
            double camX = cameraPose.getTranslation().getX();
            double camZ = cameraPose.getTranslation().getY();
            double pitch = cameraPose.getRotation().getY();
            double yaw = cameraPose.getRotation().getZ();

            double distY = tyDistanceMap.get(ty) + camY;
            // double distY = camZ * Math.tan((ty + pitch) * Math.PI / 180) + camY + robotY;
            double distX = distY * Math.tan(tx * Math.PI / 180) + camX;

            SmartDashboard.putNumber("ty", ty);
            SmartDashboard.putNumber("distY", distY);

            Translation2d robotTranslation = new Translation2d(distX, distY);
            Translation2d cameraTranslation = robotTranslation.rotateBy(new Rotation2d(yaw));
            Translation2d fieldRelativeTranslation = cameraTranslation.rotateBy(Rotation2d.kCW_90deg.plus(robotPose.getRotation()));
            Translation2d fieldTranslation = robotPose.getTranslation().plus(fieldRelativeTranslation);

            Pose2d objectPose = new Pose2d(fieldTranslation, robotPose.getRotation());

            if (classId == 0) {
                latestAlgae = objectPose;
            } else {
                latestCoral = objectPose;
            }

            detectedObjects.add(objectPose);
        }

        field.getObject("objects").setPoses(detectedObjects.stream().toList());
    }
}
