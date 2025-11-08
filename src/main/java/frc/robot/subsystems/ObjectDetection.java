package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private class DetectedObject {
        public enum ObjectType {
            ALGAE,
            CORAL
        };

        public final ObjectType objectType;
        public final Translation2d fieldTranslation;

        public DetectedObject(int classId, Translation2d translation) {
            this.fieldTranslation = translation;
            this.objectType = classId == 0 ? ObjectType.ALGAE : ObjectType.CORAL;
        }

        public String toString() {
            return "Type: " + objectType + "\nTranslation X: " + fieldTranslation.getX() + "\nTranslation Y: " + fieldTranslation.getY(); 
        }
    }

    public ObjectDetection(String name, Transform3d cameraPose, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.cameraPose = cameraPose;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        rawDetectionSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(null);
        SmartDashboard.putData("ObjectDetection", field);
    } 

    @Override
    public void periodic() {
        Pose2d robotPose = poseSupplier.get(); 
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double[] rawDetections = rawDetectionSubscriber.get();
        if (rawDetections == null) return;
        int objectCount = rawDetections.length / entriesPerObject;
        List<DetectedObject> detectedObjects = new ArrayList<>();

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



            double distY = camZ * Math.tan((ty + pitch) * Math.PI / 180) + camY + robotY;
            double distX = distY * Math.tan((tx + yaw) * Math.PI / 180) + camX + robotX;


            detectedObjects.add(new DetectedObject(classId, new Translation2d(distX, distY)));
        }


        field.getObject("objects").setPoses(detectedObjects.stream().map(obj -> new Pose2d(obj.fieldTranslation, new Rotation2d())).toList());
    }
}
