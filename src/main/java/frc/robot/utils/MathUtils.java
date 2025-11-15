package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;

public class MathUtils {
    public static Angle clamp(Angle value, Angle min, Angle max) {
        double v = value.in(Degrees);
        double lo = min.in(Degrees);
        double hi = max.in(Degrees);
        return Degrees.of(Math.max(lo, Math.min(v, hi)));
    }
}
