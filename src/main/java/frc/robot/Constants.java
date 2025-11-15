package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Millimeter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class Robot {
        public static double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                             // top speed;
        public static double MAX_ANGULAR_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);

        public static double MAX_SLOW_SPEED = MAX_SPEED / 4;
        public static double MAX_SLOW_ANGULAR_RATE = MAX_ANGULAR_RATE / 3;

        public static PIDController TRANSLATION_CONTROLLER = new PIDController(5, 0, 1);
        public static PIDController ROTATION_CONTROLLER = new PIDController(5, 0, 0);

        public static Distance TRANSLATION_THRESHOLD = Inches.of(0.05);
        public static Angle ROTATION_THRESHOLD = Degrees.of(0.05);

        public static double CONTROLLER_COOLDOWN = 0.3;
        public static double CONTROLLER_THRESHOLD = 0.2;
    }

    public static class Field {
        public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);
        public static double FIELD_LENGTH = APRIL_TAG_LAYOUT.getFieldLength();
        public static double FIELD_WIDTH = APRIL_TAG_LAYOUT.getFieldWidth();
    }

    public static class Vision {
        // stores tag layout for the current year's feild

        // assuming stores distance of cameras from center of bot
        public static Transform3d CAMERA_0_POS = new Transform3d(-0.272575, 0.2413, 0.520699,
                new Rotation3d(Degrees.of(0), Degrees.of(-32), Degrees.of(-20)));
        public static Transform3d CAMERA_1_POS = new Transform3d(0.272575, 0.2413, 0.510699,
                new Rotation3d(Degrees.of(0), Degrees.of(-32), Degrees.of(20)));

        public static double MAX_UNCERTAINTY = 0.3; // TBD
        public static double MAX_Z_ERROR = 0.2; // TBD

        // Standard deviation coefficents, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double LINEAR_STDEV_COEFF = 0.02; // Meters
        public static double ANGULAR_STDEV_COEFF = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        public static double LINEAR_STDEV_MEGATAG_2_COEFF = 0.5; // More stable than full 3D solve
        public static double ANGULAR_STDEV_MEGATAG_2_COEFF = Double.POSITIVE_INFINITY; // No rotation data available
    }

    public static class Shooter {
        public static Current MAXCURRENT = Amps.of(3);
    }

    public static class Indexer {
        public static int INDEXER_MOTOR_ID = 109;
        public static int LOADER_MOTOR_ID = 110;
    }

    public static class Elevator {
        public static int LEFT_MOTOR_ID = 20 ; 
        public static int RIGHT_MOTOR_ID = 21;  

        public static Angle MIN_POSITION = Rotations.of(0);
        public static Angle MAX_POSITION = Rotations.of(29.94);
        public static Angle POSITION_THRESHOLD = Rotations.of(2);
        public static final Angle OFFSET_AMOUNT = Rotations.of(3);
        public static final Angle DEFAULT = Rotations.of(0);
    }

    public static class Grabber {
        public static Distance INTAKE_Distance = Millimeter.of(35);
        public static Current INTAKE_VELOCITY = Amps.of(10); // Placeholder Value
        public static Current EJECT_VELOCITY = Amps.of(10); // Placeholder Value
        public static int MOTOR_ID = 19;
        public static int SENSOR_ID = 11;
    }
}
