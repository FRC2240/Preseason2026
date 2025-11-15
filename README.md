## For sysID in swerve

1. Run each sysID test once (either drive or azimuth).
2. Download the .hoot file and convert it to .wpilog via GUI or Owlet CLI
3. Put the .wpilog into sysID application
4. For each motor (that you tested), run sysID 
 - Analyis type = simple
 - Velocity=Velocity Position=Position Voltage=MotorVoltage
 - Units is rotations
 - Increase test duration
 - Feedback Analysis Gain Preset = CTRE Phoenix 6
 - Loop type = velocity
 - Note down Ks Kv Ka Kp Kd
5. Average the results and apply changes to TunerConstants
-- For Azimuth motors, use CANCoder position and velocity

ks 0.12248 + 0.08599 + 0.091184 + 0.062389 = 0.09051075 * 4
kv 0.12305 + 0.12487 + 0.12396 + 0.12244 = 0.12358 * 4
ka 0.014074 + 0.015631 + 0.02413 + 0.028803 = 0.0206595 * 4
kp 0.056201 + 0.082818 + 0 + 0 = 0.03475475 * 4
kd 0
