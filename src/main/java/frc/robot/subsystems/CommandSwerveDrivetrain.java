package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    //Adding this for PathPlanner functionality.  Not originally generated by Phoenix Tuner X.
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    //Adding this for *fun*.  Not originally generated by Phoenix Tuner X.
    private final MusicTone drivetrainMusicToneRequest = new MusicTone(0);

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner(); //Adding this for PathPlanner functionality.  Not originally generated by Phoenix Tuner X.
        //Run through the custom motor configurations after the constructor applies the standard configurations.
        setSwerveDriveCustomCurrentLimits();
        setSwerveDriveCustomVoltageLimits();
        setSwerveDriveCustomAudioConfig();
        setSwerveSteerCustomCurrentLimits();
        setSwerveSteerCustomVoltageLimits();
        setSwerveSteerCustomAudioConfig();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner(); //Adding this for PathPlanner functionality.  Not originally generated by Phoenix Tuner X.
        //Run through the custom motor configurations after the constructor applies the standard configurations.
        setSwerveDriveCustomCurrentLimits();
        setSwerveDriveCustomVoltageLimits();
        setSwerveDriveCustomAudioConfig();
        setSwerveSteerCustomCurrentLimits();
        setSwerveSteerCustomVoltageLimits();
        setSwerveSteerCustomAudioConfig();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        //Need to look at this a little more before I understand it.
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose, 
            this::seedFieldRelative, 
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            () -> {
                //Boolean supplier that controls when the path will be mirrored for the BLUE alliance.
                //Normally, the switch would flip from blue to red, but we built the red alliance for our practice field, and have written all of our auton routines as red so far.
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return false;   //Commenting out for the moment, since it seems to be more accurate to create auton routines for each side separately.
                    //alliance.get() == DriverStation.Alliance.Blue;
                }
                return false;
            },
            this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    //Adding this for PathPlanner functionality.  Not originally generated by Phoenix Tuner X.
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    //Adding this for PathPlanner functionality.  Not originally generated by Phoenix Tuner X.
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
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
     * Set custom current limits for the swerve drive motors.
     * The constructor already sets the stator current limit, but this method sets the supply current limit.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveDriveCustomCurrentLimits() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            //Refresh the current configuration, since the stator current limit has already been set.
            currentConfigurator.refresh(customCurrentLimitConfigs);

            //Set all of the parameters related to the supply current.  The values should come from Constants.
            customCurrentLimitConfigs.SupplyCurrentLimit = Constants.kSwerveDriveSupplyCurrentLimit;
            customCurrentLimitConfigs.SupplyCurrentLimitEnable = Constants.kSwerveDriveSupplyCurrentLimitEnable;
            customCurrentLimitConfigs.SupplyCurrentThreshold = Constants.kSwerveDriveSupplyCurrentThreshold;
            customCurrentLimitConfigs.SupplyTimeThreshold = Constants.kSwerveDriveSupplyTimeThreshold;

            //Apply the new current limit configuration.
            currentConfigurator.apply(customCurrentLimitConfigs);
        }
    }

    /**
     * Set custom current limits for the swerve steer motors.
     * The constructor already sets the stator current limit, but this method sets the supply current limit.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveSteerCustomCurrentLimits() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current steer motor.
            var currentConfigurator = module.getSteerMotor().getConfigurator();

            //Refresh the current configuration, since the stator current limit has already been set.
            currentConfigurator.refresh(customCurrentLimitConfigs);

            //Set all of the parameters related to the supply current.  The values should come from Constants.
            customCurrentLimitConfigs.SupplyCurrentLimit = Constants.kSwerveSteerSupplyCurrentLimit;
            customCurrentLimitConfigs.SupplyCurrentLimitEnable = Constants.kSwerveSteerSupplyCurrentLimitEnable;
            customCurrentLimitConfigs.SupplyCurrentThreshold = Constants.kSwerveSteerSupplyCurrentThreshold;
            customCurrentLimitConfigs.SupplyTimeThreshold = Constants.kSwerveSteerSupplyTimeThreshold;

            //Apply the new current limit configuration.
            currentConfigurator.apply(customCurrentLimitConfigs);
        }
    }

    /**
     * Set custom voltage limits for the swerve drive motors.
     * This method should not interfere with any other configurations set in the super implementation of the constructor.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveDriveCustomVoltageLimits() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customVoltageLimitConfigs = new VoltageConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            //Refresh the voltage configuration, since there may be voltage parameters that have already been set.
            currentConfigurator.refresh(customVoltageLimitConfigs);

            //Set all of the parameters related to the forward and reverse voltages.  The values should come from Constants.
            customVoltageLimitConfigs.PeakForwardVoltage = Constants.kSwerveDriveMaxForwardVoltage;
            customVoltageLimitConfigs.PeakReverseVoltage = Constants.kSwerveDriveMaxReverseVoltage;

            //Apply the new voltage configuration.
            currentConfigurator.apply(customVoltageLimitConfigs);
        }
    }

    /**
     * Set custom voltage limits for the swerve steer motors.
     * This method should not interfere with any other configurations set in the super implementation of the constructor.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveSteerCustomVoltageLimits() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customVoltageLimitConfigs = new VoltageConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current steer motor.
            var currentConfigurator = module.getSteerMotor().getConfigurator();

            //Refresh the voltage configuration, since there may be voltage parameters that have already been set.
            currentConfigurator.refresh(customVoltageLimitConfigs);

            //Set all of the parameters related to the forward and reverse voltages.  The values should come from Constants.
            customVoltageLimitConfigs.PeakForwardVoltage = Constants.kSwerveSteerMaxForwardVoltage;
            customVoltageLimitConfigs.PeakReverseVoltage = Constants.kSwerveSteerMaxReverseVoltage;

            //Apply the new voltage configuration.
            currentConfigurator.apply(customVoltageLimitConfigs);
        }
    }     

    /**
     * Set custom audio configurations for the swerve drive motors.
     * Specifically, this allows the motors to play music while disabled.
     * This method should not interfere with any other configurations set in the super implementation of the constructor.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveDriveCustomAudioConfig() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customAudioConfigs = new AudioConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            //Refresh the audio configuration, since there may be audio parameters that have already been set.
            currentConfigurator.refresh(customAudioConfigs);

            //Allow the motors to play music while they are disabled.
            customAudioConfigs.AllowMusicDurDisable = true;

            //Apply the new audio configuration.
            currentConfigurator.apply(customAudioConfigs);
        }
    }

    /**
     * Set custom audio configurations for the swerve steer motors.
     * Specifically, this allows the motors to play music while disabled.
     * This method should not interfere with any other configurations set in the super implementation of the constructor.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveSteerCustomAudioConfig() {
        //Create a current configuration to use for the steer motor of each swerve module.
        var customAudioConfigs = new AudioConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current steer motor.
            var currentConfigurator = module.getSteerMotor().getConfigurator();

            //Refresh the audio configuration, since there may be audio parameters that have already been set.
            currentConfigurator.refresh(customAudioConfigs);

            //Allow the motors to play music while they are disabled.
            customAudioConfigs.AllowMusicDurDisable = true;

            //Apply the new audio configuration.
            currentConfigurator.apply(customAudioConfigs);
        }
    }

   /**
   * Add all of the motors in the drivetrain to the Orchestra.
   * I want the robot to sing.
   * @param robotOrchestra The Orchestra to add the motors as instruments to.
   */
  public void addToOrchestra(Orchestra robotOrchestra) {
    //Iterate through each module.
    for (var module : Modules) {
        robotOrchestra.addInstrument(module.getDriveMotor(), 6);
        robotOrchestra.addInstrument(module.getSteerMotor(), 6);
    }
  }   

   /**
   * Sets all motors in the drivetrain to play a tone at the requested frequency.
   * @param toneInput A percentage, mapped to kMusicToneTable lookup table.
   */
  public void setMusicToneFrequency(double toneInput) {
    for (var module : Modules) {
        module.getDriveMotor().setControl(drivetrainMusicToneRequest.withAudioFrequency(Constants.kMusicToneTable.get(toneInput)));
        module.getSteerMotor().setControl(drivetrainMusicToneRequest.withAudioFrequency(Constants.kMusicToneTable.get(toneInput)));
    }
  } 

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
