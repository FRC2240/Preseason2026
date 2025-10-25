package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class Robot {
        // kSpeedAt12Volts desired top speed;
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

        public static double MaxSlowSpeed = MaxSpeed / 4;
        public static double MaxSlowAngularRate = MaxAngularRate / 3;

        public static PIDController TranslationController = new PIDController(5, 0, 1);
        public static PIDController RotationController = new PIDController(5, 0, 0);
        public static Distance TranslationDeadband = Inches.of(0.05);
        public static Angle RotationDeadband = Degrees.of(0.05);
        public static double ControllerCooldown = 0.3;
        public static double ControllerThreshold = 0.2;
    }
}
