package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private Logger logger = new Logger(Constants.Robot.MAX_SPEED);

    /* Driving configs */
    private CommandXboxController joystick;
    private Timer joystickTimer = new Timer();

    private final SwerveRequest.ApplyRobotSpeeds pathplannerApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds driveAlignment = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Robot.MAX_SPEED * 0.1)
            .withRotationalDeadband(Robot.MAX_ANGULAR_RATE * 0.2)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SmartDashboard.putString("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    this::logSysId,
                    this));

    private void logSysId(SysIdRoutineLog log) {
        var modules = getModules();
        for (int i = 0; i < 4; i++) {
            var module = modules[i];
            TalonFX driveMotor = module.getDriveMotor();
            TalonFX angleMotor = module.getSteerMotor();
            CANcoder encoder = module.getEncoder();

            log.motor("drive-motor-" + i).voltage(driveMotor.getMotorVoltage().getValue()).angularPosition(driveMotor.getPosition().getValue()).angularVelocity(driveMotor.getVelocity().getValue());
        }
    }
    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* Constructor */
    public Drivetrain(CommandXboxController joystick) {
        // Uses TunerConstants values directly
        // For more config, check all of the overrides for super
        super(TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);

        this.joystick = joystick;

        registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        setupPathPlanner();
    }

    private void setupPathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            pathplannerApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }

    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }


    private Alliance currentAlliancePerspective = Alliance.Red;

    @Override
    public void periodic() {
        /*
         * Undefined/Red -> Red
         * Undefined/Red -> Blue
         * Red -> Blue
         * Blue -> Red
         */

         // This operation should only be preformed in disabled for safetey reasons
        if (DriverStation.isEnabled()) return;

        Optional<Alliance> rawAlliance = DriverStation.getAlliance();
        Alliance alliance = rawAlliance.isPresent()? rawAlliance.get() : Alliance.Red;

        if (!currentAlliancePerspective.equals(alliance)) {
            // Flip gyro by 180
            var currentYaw = getPigeon2().getYaw().getValue();
            getPigeon2().setYaw(currentYaw.plus(Degrees.of(180)));
        }
    }


    public Command zeroGyroWithAllianceCommand() {
        /*
         * Does not set the drivetrain as a requrirement because
         * that might override driving and that wouldnt be good.
         * 
         * Ignores disabled to be able to do it in disabled
         */
        return Commands.runOnce(() -> {
            getPigeon2().setYaw(currentAlliancePerspective == Alliance.Red ? 0 : 180);
        }).ignoringDisable(true);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public Command idleCommand() {
        return applyRequest(() -> idle).ignoringDisable(true);
    }

    public Rotation2d getHeading() {
        return getState().RawHeading;
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Command driveCommand() {
        return applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * Robot.MAX_SPEED)
                .withVelocityY(-joystick.getLeftX() * Robot.MAX_SPEED)
                .withRotationalRate(-joystick.getRightX() * Robot.MAX_ANGULAR_RATE));
    }

    public Command driveSlowCommand() {
        return applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * Robot.MAX_SLOW_SPEED)
                .withVelocityY(-joystick.getLeftX() * Robot.MAX_SLOW_SPEED)
                .withRotationalRate(-joystick.getRightX() * Robot.MAX_SLOW_ANGULAR_RATE));
    }

    public Command driveToPose(Pose2d target) {
        return Commands.runOnce(() -> {
            joystickTimer.restart();
        }, this).andThen(Commands.run(() -> {
            Pose2d currentPose = getState().Pose;

            // Gets x and y components
            Translation2d translationToTarget = target.getTranslation().minus(currentPose.getTranslation());
            Rotation2d directionOfTravel = translationToTarget.getAngle();
            double linearDistance = translationToTarget.getNorm();

            double velocityOutput = Math.min(Robot.TRANSLATION_CONTROLLER.calculate(linearDistance, 0),
                    Robot.MAX_SPEED);
            LinearVelocity xComponent = MetersPerSecond.of(-velocityOutput * directionOfTravel.getCos());
            LinearVelocity yComponent = MetersPerSecond.of(-velocityOutput * directionOfTravel.getSin());

            // Gets angle
            double rotationDistRadians = target.getRotation().minus(currentPose.getRotation()).getRadians();
            AngularVelocity omega = RadiansPerSecond
                    .of(MathUtil.clamp(-Robot.ROTATION_CONTROLLER.calculate(rotationDistRadians, 0),
                            -Robot.MAX_ANGULAR_RATE, Robot.MAX_ANGULAR_RATE));

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xComponent, yComponent, omega, currentPose.getRotation());

            setControl(driveAlignment.withSpeeds(speeds));
        }, this).until(() -> {
            // Until close enough to the pose
            Pose2d currentPose = getState().Pose;

            return currentPose.getRotation().getMeasure().isNear(target.getRotation().getMeasure(),
                    Robot.ROTATION_THRESHOLD) &&
                    currentPose.getTranslation().getMeasureX().isNear(target.getTranslation().getMeasureX(),
                            Robot.TRANSLATION_THRESHOLD)
                    &&
                    currentPose.getTranslation().getMeasureY().isNear(target.getTranslation().getMeasureY(),
                            Robot.TRANSLATION_THRESHOLD);
        }).until(() -> {
            return joystickTimer.hasElapsed(Robot.CONTROLLER_COOLDOWN) &&
                    (Math.abs(joystick.getRightX()) > Robot.CONTROLLER_THRESHOLD ||
                            Math.abs(joystick.getRightY()) > Robot.CONTROLLER_THRESHOLD ||
                            Math.abs(joystick.getLeftX()) > Robot.CONTROLLER_THRESHOLD ||
                            Math.abs(joystick.getLeftY()) > Robot.CONTROLLER_THRESHOLD);
        }));
    }
}