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

ks 0.097106 + 0.14069 + 0.073032 = 0.103609
kv 2.6777 + 2.7182 + 2.7039 = 2.69993
ka 0.4244 + 0.43814 + 0.45285 = 0.438463
kp 0.013796 + 3.566 + 0 = 1.193265
