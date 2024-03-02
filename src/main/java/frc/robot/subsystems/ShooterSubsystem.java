// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The shooter subsystem takes a note that was picked up off the ground, and puts it somewhere that isn't the ground.
 * <p><i>Maybe.</i>
 */
public class ShooterSubsystem extends SubsystemBase {
 
  //Make variables
  public String m_shooterMode = "DoNothing";
  public boolean m_readyToShoot = false;
  public double m_desiredPosition = 0;
  public double m_desiredSpeed = 0;

  
  TalonFX m_ShooterArmRotateLeft = new TalonFX(Constants.kShooterArmRotateLeftCanID, "Canivore");
  TalonFX m_ShooterArmRotateRight = new TalonFX(Constants.kShooterArmRotateRightCanID, "Canivore");
  TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");
  TalonFX m_SpinRightShooter = new TalonFX(Constants.kSpinRightShooterCanID, "Canivore");
  TalonFX m_SpinLeftShooter = new TalonFX(Constants.kSpinLeftShooterCanID, "Canivore"); 

  //Open CANcoder
  CANcoder m_ShooterArmCANcoder = new CANcoder(Constants.kShooterArmCancoderCanID);
  
  private final VelocityVoltage ShooterVelocity = new VelocityVoltage(0.0, 0.0, true, 0,0, false, false, false);
  //private final MotionMagicVoltage ShooterArmPosition = new MotionMagicVoltage(0.0, true, 0, 0, false, false, false);
  private final MotionMagicVoltage ShooterArmPositionRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false).withSlot(0);

  /**
   * Constructs an instance of the shooter subsystem.
   * The motor and sensor configuration is done here.
   */
  public ShooterSubsystem() {
    //Configure Motors in seperate methods for clarity
    this.configureShooterArmCANcoder(m_ShooterArmCANcoder);
    this.configureArmRotate(m_ShooterArmRotateLeft, m_ShooterArmRotateRight);
    this.configureSpinRightShooter(m_SpinRightShooter);
    this.configureSpinLeftShooter(m_SpinLeftShooter);
  }

  /**
   * Sets whether the shooter is ready to shoot a note.
   * @param readyToShoot Whether the shooter is ready to shoot a note or not.
   * True or false.
   */
  public void setShooterReadyToShoot(boolean readyToShoot) {
    System.out.println("ShooterSubsystem: readyToShoot");
    m_readyToShoot = readyToShoot;
  }

  /**
   * Checks if the shooter is ready to shoot a note.
   * @return Whether the shooter is ready to shoot a note or not.
   * True or false.
   */
  public boolean getShooterReadyToShoot() {
    System.out.println("ShooterSubsystem: getShooterReadyToShoot");
    return m_readyToShoot;
  }

  // Start of motor set methods

  /**
   * Sets the current shooter mode.  Acceptable values are:
   * <p>ShootAmp
   * <p>ShootTrap
   * <p>ShootPodium
   * <p>ShootSubwoofer
   * <p>ShootWithPose
   * @param shooterMode The mode to put the shooter into.
   */
  public void setShooterMode(String shooterMode){
    m_shooterMode = shooterMode;
    System.out.println("ShooterSubsystem: setShooterMode");
  }

  /**
   * Sets the speed of the shooter motors.
   * The left shooter motor is set to the input speed.
   * The right shooter motor is set to 90% of the input speed.
   * @param shootSpeed The desired speed of the shooter motors, in rotations per second.
   */
  public void setShooterSpeed(double shootSpeed) {
    System.out.println("ShooterSubsystem: setShooterSpeed");
    m_SpinLeftShooter.setControl(ShooterVelocity.withVelocity(shootSpeed));
    m_SpinRightShooter.setControl(ShooterVelocity.withVelocity((shootSpeed)));
    m_desiredSpeed = shootSpeed;
  }

  /**
   * Sets the position of the shooter arm.
   * @param position The position of the shooter arm, in rotations.
   */
  public void setShooterArmPosition(double position) {
    if (Constants.kMotorEnableShooterArmRotate ==1){
      System.out.println("ShooterSubsystem: setShooterArmPosition");
      m_ShooterArmRotateLeft.setControl(ShooterArmPositionRequest.withPosition(position));
    }
    m_desiredPosition = position;
  }

  /**
   * Get the current shooter mode.
   * @return The currently active shooter mode, as a String.
   */
  public String getShooterMode(){
    System.out.println("ShooterSubsystem: getShooterMode");
    return m_shooterMode;
  }

  /**
   * Get the current speed of the left shooter motor.
   * @return The current speed of the left shooter motor, in rotations per second.
   */
  public double getShooterSpeed() {
    System.out.println("ShooterSubsystem: getShooterSpeed");
    m_SpinLeftShooter.getVelocity().getValueAsDouble();
    return m_SpinLeftShooter.getVelocity().getValueAsDouble();

  }

  /**
   * Get the current position of the shooter arm motor.
   * @return The current position of the shooter arm, in rotations.
   */
  public double getShooterArmPosition() {
    System.out.println("ShooterSubsystem: getShooterArmPosition");
    return m_ShooterArmRotateLeft.getPosition().getValueAsDouble();
  }

  /**
   * Check if the current shooter motor speed is within tolerance of the desired speed.
   * @param desiredSpeed The shooter motor speed to check, in rotations per second.
   * @return Whether the shooter motor is at the desired speed.
   * True or false.
   */
  public boolean getShooterUpToSpeed(double desiredSpeed) {
    System.out.println("ShooterSubsystem: getShooterUpToSpeed");
    return ((this.getShooterSpeed() >= (desiredSpeed - Constants.kShooterSpeedTolerance)) && (this.getShooterSpeed() <= (desiredSpeed + Constants.kShooterSpeedTolerance)));
  }

  /**
   * Check if the current shooter motor speed is within tolerance of the desired speed.
   * This is an overload.
   * If the desired speed is not passed to this method, it will use the last speed set with {@link #setShooterSpeed}.
   * @return Whether the shooter motor is at the desired speed.
   */
  public boolean getShooterUpToSpeed() {
    //return this.getShooterUpToSpeed(m_desiredSpeed);
    return false;
  }

  /**
   * Check if the current shooter arm position is within tolerance of the desired position.
   * @param desiredPosition The shooter arm position to check, in rotations.
   * @return Whether the shooter arm is at the desired position.
   * True or false.
   */
  public Boolean getShooterArmInPosition(double desiredPosition) {
      System.out.println("ShooterSubsystem: getShooterArmInPosition");
      return ((this.getShooterArmPosition() > (desiredPosition - Constants.kShooterArmTolerancePos)) && (this.getShooterArmPosition() < (desiredPosition + Constants.kShooterArmTolerancePos)));

    }

  /**
   * Check if the current shooter arm position is within tolerance of the desired position.
   * This is an overload.
   * If the desired position is not passed to this method, it will use the last position set with {@link #setShooterArmPosition}.
   * @return Whether the shooter arm is at the desired position.
   * True or false.
   */
  public Boolean getShooterArmInPosition() {
    return this.getShooterArmInPosition(m_desiredPosition);
  }
    //End of motor get methods

  public void SetShooterArmLeftNeutralMode(NeutralModeValue NeutralMode) {
    var neuMotOut = new MotorOutputConfigs();
    var currentConfigurator = m_ShooterArmRotateLeft.getConfigurator();
    currentConfigurator.refresh(neuMotOut);
    neuMotOut.NeutralMode = NeutralMode;
    currentConfigurator.apply(neuMotOut);
  }
  
  public void SetShooterArmRightNeutralMode(NeutralModeValue NeutralMode) {
    var neuMotOut = new MotorOutputConfigs();
    var currentConfigurator = m_ShooterArmRotateRight.getConfigurator();
    currentConfigurator.refresh(neuMotOut);
    neuMotOut.NeutralMode = NeutralMode;
    currentConfigurator.apply(neuMotOut);
  }

    /**
   * Open Motors
   * 
   */

  public void configureShooterArmCANcoder(CANcoder shooterArmCANcoder){
    //Configuring CANcoder
    CANcoderConfiguration ShooterArmCANcoderConfig = new CANcoderConfiguration();
    ShooterArmCANcoderConfig.MagnetSensor.MagnetOffset = 0.0;
    ShooterArmCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    ShooterArmCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
  }



  public void configureArmRotate(TalonFX shooterArmRotateLeft, TalonFX shooterArmRotateRight){

   TalonFXConfiguration ShooterArmRotateConfig = new TalonFXConfiguration();
    
    ShooterArmRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kShooterArmRotateSupplyCurrentLimit;
    ShooterArmRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kShooterArmRotateVoltageClosedLoopRampPeriod;
    ShooterArmRotateConfig.MotionMagic.MotionMagicAcceleration = Constants.kShooterArmRotateMotionMagicAcceleration;
    ShooterArmRotateConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kShooterArmRotateMotionMagicCruiseVelocity;
    //ShooterArmRotateConfig.MotionMagic.MotionMagicJerk = Constants.kShooterArmRotateMotionMagicJerk; Idk if we want Jerk
    ShooterArmRotateConfig.Voltage.PeakForwardVoltage = Constants.kShooterArmRotateMaxForwardVoltage;
    ShooterArmRotateConfig.Voltage.PeakReverseVoltage = Constants.kShooterArmRotateMaxReverseVoltage;
    ShooterArmRotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ShooterArmRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0 = ShooterArmRotateConfig.Slot0;
    slot0.kP = Constants.kShooterArmRotateProportional;
    slot0.kI = Constants.kShooterArmRotateIntegral;
    slot0.kD = Constants.kShooterArmRotateDerivative;
    slot0.kG = Constants.kShooterArmRotateGravity;
    slot0.kV = Constants.kShooterArmRotateVelocityFeedFoward;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    //slot0.kV = Constants.kShooterArmRotateVelocityFeedFoward;
    //slot0.kS = Constants.kShooterArmRotateStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving
    ShooterArmRotateConfig.MotorOutput.Inverted = Constants.kShooterArmRotateDirection;

    // Pretty sure I properly added fused cancoder here
    ShooterArmRotateConfig.Feedback.FeedbackRemoteSensorID = Constants.kShooterArmCancoderCanID;
    ShooterArmRotateConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    ShooterArmRotateConfig.Feedback.RotorToSensorRatio = Constants.kShooterArmRotateCancoderRotorToSensorRatio;
    
    StatusCode shooterArmStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      shooterArmStatus = m_ShooterArmRotateLeft.getConfigurator().apply(ShooterArmRotateConfig);
      if (shooterArmStatus.isOK()) break;
    }
    if (!shooterArmStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + shooterArmStatus.toString());
    }
    m_ShooterArmRotateRight.setControl(new Follower(Constants.kShooterArmRotateLeftCanID, true));
  }



  public void configureSpinRightShooter(TalonFX spinRightShooter){
    TalonFXConfiguration RightShooterMotorsConfig = new TalonFXConfiguration();

    RightShooterMotorsConfig.Slot0.kP = Constants.kRightShooterProportional; // An error of 1 rotation per second results in 2V output
    RightShooterMotorsConfig.Slot0.kI = Constants.kRightShooterIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    RightShooterMotorsConfig.Slot0.kD = Constants.kRightShooterDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    RightShooterMotorsConfig.Slot0.kV = Constants.kRightShooterVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  
    RightShooterMotorsConfig.MotorOutput.Inverted = Constants.kRightShooterDirection;

    // Peak output of 8 volts
    RightShooterMotorsConfig.Voltage.PeakForwardVoltage = Constants.kRightShooterMaxForwardVoltage;
    RightShooterMotorsConfig.Voltage.PeakReverseVoltage = Constants.kRightShooterMaxReverseVoltage;
    RightShooterMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    RightShooterMotorsConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightShooterSupplyCurrentLimit;
    RightShooterMotorsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightShooterVoltageClosedLoopRampPeriod;
    
    StatusCode rightShooterStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      rightShooterStatus = m_SpinRightShooter.getConfigurator().apply(RightShooterMotorsConfig);
      if (rightShooterStatus.isOK()) break;
    }
    if (!rightShooterStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + rightShooterStatus.toString());
    }
  }

  public void configureSpinLeftShooter(TalonFX spinLeftShooter){
    TalonFXConfiguration LeftShooterMotorsConfig = new TalonFXConfiguration();

    LeftShooterMotorsConfig.Slot0.kP = Constants.kLeftShooterProportional; // An error of 1 rotation per second results in 2V output
    LeftShooterMotorsConfig.Slot0.kI = Constants.kLeftShooterIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    LeftShooterMotorsConfig.Slot0.kD = Constants.kLeftShooterDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    LeftShooterMotorsConfig.Slot0.kV = Constants.kLeftShooterVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  
    LeftShooterMotorsConfig.MotorOutput.Inverted = Constants.kLeftShooterDirection;

    // Peak output of 8 volts
    LeftShooterMotorsConfig.Voltage.PeakForwardVoltage = Constants.kLeftShooterMaxForwardVoltage;
    LeftShooterMotorsConfig.Voltage.PeakReverseVoltage = Constants.kLeftShooterMaxReverseVoltage;
    LeftShooterMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    LeftShooterMotorsConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftShooterSupplyCurrentLimit;
    LeftShooterMotorsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftShooterVoltageClosedLoopRampPeriod;
    
    StatusCode leftShooterStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      leftShooterStatus = m_SpinLeftShooter.getConfigurator().apply(LeftShooterMotorsConfig);
      if (leftShooterStatus.isOK()) break;
    }
    if (!leftShooterStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + leftShooterStatus.toString());
    }
  }




  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ShooterSubsystem");
    builder.addStringProperty("Shooter Mode", this::getShooterMode, null);
    builder.addBooleanProperty("Ready to Shoot?", this::getShooterReadyToShoot, null);
    builder.addDoubleProperty("Shooter Arm Position", this::getShooterArmPosition, null);
    builder.addDoubleProperty("Shooter Speed", this::getShooterSpeed, null);
    builder.addBooleanProperty("Shooter Up to Speed?", this::getShooterUpToSpeed, null);
    builder.addBooleanProperty("Shooter Arm in Position?", this::getShooterArmInPosition, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
