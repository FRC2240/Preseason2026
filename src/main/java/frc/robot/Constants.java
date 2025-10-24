package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static class Robot {
        // kSpeedAt12Volts desired top speed;
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
        public static double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); 

        public static double MaxSlowSpeed = MaxSpeed / 3;
        public static double MaxSlowAngularRate = MaxAngularRate / 3;
    }
}
