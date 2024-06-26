// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * The front intake subsystem sucks up notes from the floor and into the robot.
 * <p><i>Nom-nom-nom. Rar.  Wooooooooo</i>
 */
public class FrontIntakeSubsystem extends SubsystemBase {
  
  TalonFX m_frontIntakeSpin = new TalonFX(Constants.kFrontIntakeSpinCanID, "Canivore");
  TalonFX m_frontIntakeRotate = new TalonFX(Constants.kFrontIntakeRotateCanID, "Canivore");
  CANcoder m_frontIntakeRotateCANcoder = new CANcoder(Constants.kFrontIntakeRotateCancoderCanID);
  Servo m_frontIntakeServo = new Servo(0);

  private final VelocityVoltage frontIntakeVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final MotionMagicVoltage frontIntakeRotationRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false, false, false);
  private final DutyCycleOut frontIntakeVoltageRequest = new DutyCycleOut(0, false, false, false, false);
  
  private final MusicTone frontIntakeMusicToneRequest = new MusicTone(0);

  public double m_desiredPosition = 0;
  public double m_desiredSpeed = 0;

  /**
   * Constructs an instance of the front intake subsystem.
   * The motor and sensor configuration is done here.
   */
  public FrontIntakeSubsystem() {
    this.configureFrontIntakeCancoder(m_frontIntakeRotateCANcoder);
    this.configureFrontIntakeSpin(m_frontIntakeSpin);
    this.configureFrontIntakeRotate(m_frontIntakeRotate);
  }

  /**
   * Sets the position of the front intake.
   * @param position The desired position of the front intake, in rotations.
   */

  public void configureFrontIntakeRotate(TalonFX frontIntakeSpin){
    TalonFXConfiguration frontIntakeRotateConfig = new TalonFXConfiguration();

    frontIntakeRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kFrontIntakeRotateSupplyCurrentLimit;
    frontIntakeRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kFrontIntakeRotateVoltageClosedLoopRampPeriod;
    frontIntakeRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    frontIntakeRotateConfig.MotorOutput.Inverted = Constants.kFrontIntakeRotateDirection;
    frontIntakeRotateConfig.Voltage.PeakForwardVoltage = Constants.kFrontIntakeRotateMaxForwardVoltage;
    frontIntakeRotateConfig.Voltage.PeakReverseVoltage = Constants.kFrontIntakeRotateMaxReverseVoltage;
    frontIntakeRotateConfig.MotionMagic.MotionMagicAcceleration = Constants.kFrontIntakeRotateMotionMagicAcceleration;
    frontIntakeRotateConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kFrontIntakeRotateMotionMagicCruiseVelocity;
    //frontIntakeRotateConfig.MotionMagic.MotionMagicJerk = Constants.kFrontIntakeRotateMotionMagicJerk; // Idk how needed Jerk is
    frontIntakeRotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0 = frontIntakeRotateConfig.Slot0;
    slot0.kP = Constants.kFrontIntakeRotateProportional;
    slot0.kI = Constants.kFrontIntakeRotateIntegral;
    slot0.kD = Constants.kFrontIntakeRotateDerivative;
    slot0.kG = Constants.kFrontIntakeRotateGravity;
    slot0.kV = Constants.kFrontIntakeRotateVelocityFeedFoward;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    //slot0.kV = Constants.kFrontIntakeRotateVelocityFeedFoward;
    //slot0.kS = Constants.kFrontIntakeRotateStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving
    frontIntakeRotateConfig.Feedback.FeedbackRemoteSensorID = Constants.kFrontIntakeRotateCancoderCanID;
    frontIntakeRotateConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    frontIntakeRotateConfig.Feedback.RotorToSensorRatio = Constants.kFrontIntakeRotateRotorToSensorRatio;

    //Setting the config option that allows playing music on the motor during disabled.
    frontIntakeRotateConfig.Audio.AllowMusicDurDisable = true;

    StatusCode frontIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      frontIntakeRotateStatus = m_frontIntakeRotate.getConfigurator().apply(frontIntakeRotateConfig);
      if (frontIntakeRotateStatus.isOK()) break;
    }
    if (!frontIntakeRotateStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + frontIntakeRotateStatus.toString());
    }
  }

  public void configureFrontIntakeSpin(TalonFX frontIntakeSpin){
    TalonFXConfiguration frontIntakeSpinVelocityConfig = new TalonFXConfiguration();
    frontIntakeSpinVelocityConfig.Slot0.kP = Constants.kFrontIntakeSpinProportional; // An error of 1 rotation per second results in 2V output
    frontIntakeSpinVelocityConfig.Slot0.kI = Constants.kFrontIntakeSpinIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    frontIntakeSpinVelocityConfig.Slot0.kD = Constants.kFrontIntakeSpinDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    frontIntakeSpinVelocityConfig.Slot0.kV = Constants.kFrontIntakeSpinVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    frontIntakeSpinVelocityConfig.Voltage.PeakForwardVoltage = Constants.kFrontIntakeSpinMaxForwardVoltage;
    frontIntakeSpinVelocityConfig.Voltage.PeakReverseVoltage = Constants.kFrontIntakeSpinMaxReverseVoltage;
    frontIntakeSpinVelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kFrontIntakeSpinSupplyCurrentLimit;
    frontIntakeSpinVelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kFrontIntakeSpinVoltageClosedLoopRampPeriod;
    frontIntakeSpinVelocityConfig.MotorOutput.Inverted = Constants.kFrontIntakeSpinDirection;
    frontIntakeSpinVelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    //Setting the config option that allows playing music on the motor during disabled.
    frontIntakeSpinVelocityConfig.Audio.AllowMusicDurDisable = true;

    StatusCode frontIntakeSpinStatus = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      frontIntakeSpinStatus = frontIntakeSpin.getConfigurator().apply(frontIntakeSpinVelocityConfig);
      if (frontIntakeSpinStatus.isOK()) break;
    }
    if (!frontIntakeSpinStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + frontIntakeSpinStatus.toString());
    }
  }
  
  public void configureFrontIntakeCancoder(CANcoder frontIntakeCancoder){  
    CANcoderConfiguration frontIntakeRotateCANcoderConfig = new CANcoderConfiguration();
    frontIntakeRotateCANcoderConfig.MagnetSensor.MagnetOffset = -0.398926;
    frontIntakeRotateCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    frontIntakeRotateCANcoderConfig.MagnetSensor.SensorDirection = Constants.kFrontIntakeRotateCancoderDirection;
  }

  /**
   * Sets the speed of the front intake roller.
   * @param speed The desired speed of the front intake roller, in rotations per second.
   */
  public void setFrontIntakeSpeed(double speed) {
    if (Constants.kMotorEnableFrontIntakeSpin == 1){
      m_frontIntakeSpin.setControl(frontIntakeVelocityRequest.withVelocity(speed));
      if (Constants.kPrintSubsystemFrontIntake){System.out.println("FrontIntakeSubsystem - setFrontIntakeSpeed");}
    }
    m_desiredSpeed = speed;
  }

  public void setFrontIntakePosition(double position) {
    //Check if motor safety is turned on or off
    if ((Constants.kMotorEnableFrontIntakeRotate==1)&& (Robot.robotClimbed == false)){

      //Check if motor is within soft stop range
      if ((position >= Constants.kFrontIntakeDownSafety) && (position <= Constants.kFrontIntakeUpSafety)) {
        m_frontIntakeRotate.setControl(frontIntakeRotationRequest.withPosition(position));
        if (Constants.kPrintSubsystemFrontIntake){System.out.println("FrontIntakeSubsystem - setFrontIntakePosition");}
      }
    }
    m_desiredPosition = position;
  }

  public double getAbsolutePosition(){
    return (m_frontIntakeRotateCANcoder.getAbsolutePosition().getValue()-Constants.KFrontIntakeCancoderOffset);
  }

  /**
   * Get the current speed of the front intake roller.
   * @return The speed of the front intake roller motor, in rotations per second.
   */
  public double getFrontIntakeSpeed() {
    if (Constants.kPrintSubsystemFrontIntake){System.out.println("FrontIntakeSubsystem: getFrontIntakeSpeed");}
    return m_frontIntakeSpin.getVelocity().getValueAsDouble();
  }

  /**
   * Get the current position of the front intake.
   * @return The current position of the front intake, in rotations.
   */
  public double getFrontIntakePosition() {
    if (Constants.kPrintSubsystemFrontIntake){System.out.println("FrontIntakeSubsystem: getFrontIntakePosition");}
    return m_frontIntakeRotate.getPosition().getValueAsDouble();
  }
  
  /**
   * Check if the current front intake position is within tolerance of the desired position.
   * @param desiredPosition The front intake position to check, in rotations.
   * @return Whether the shooter arm is at the desired position.
   * True or false.
   */
  public Boolean getFrontIntakeInPosition(double desiredPosition) {
    if (Constants.kPrintSubsystemFrontIntake){System.out.println("FrontIntakeSubsystem - getFrontIntakeInPosition");}
    return ((this.getFrontIntakePosition() > (desiredPosition - Constants.kFrontIntakeTolerancePos)) && (this.getFrontIntakePosition() < (desiredPosition + Constants.kFrontIntakeTolerancePos)));
  }

  public Boolean getFrontIntakeIsClear() {
    if ((m_frontIntakeRotate.getPosition().getValueAsDouble()) > (Constants.kFrontIntakeClearPos - Constants.kFrontIntakeTolerancePos)) {
      return true;
    } else {
      return false;
    }
  }

  public void setFrontIntakeRotateZeroOutput() {
    m_frontIntakeRotate.setControl(frontIntakeVoltageRequest);
  }

  /**
   * Check if the current front intake position is within tolerance of the desired position.
   * This is an overload.
   * If the desired position is not passed to this method, it will use the last position set with {@link #setFrontIntakePosition}.
   * @return Whether the front intake is at the desired position.
   * True or false.
   */
  public Boolean getFrontIntakeInPosition() {
    return this.getFrontIntakeInPosition(m_desiredPosition);
  }

  public void SetFrontIntakeRotateNeutralMode(NeutralModeValue NeutralMode) {
    var neuMotOut = new MotorOutputConfigs();
    var currentConfigurator = m_frontIntakeRotate.getConfigurator();
    currentConfigurator.refresh(neuMotOut);
    neuMotOut.NeutralMode = NeutralMode;
    currentConfigurator.apply(neuMotOut);
  }

  /**
   * Set the position of the front intake servo.
   * The position is a percentage of the full range of the servo.
   * @param servoPos The desired position of the front intake servo.
   * From 0.0 to 1.0.
   */
  public void setServoPosition(double servoPos) {
    // frontIntakeServo.set(servoPos);
  }

  /**
   * Get the current position of the front intake servo.
   * @return The current position, as a percentage of the full range of the servo.
   * From 0.0 to 1.0.
   */
  public double getServoPosition() {
    // return frontIntakeServo.getPosition();
    return 10.5;
  }

  /**
   * Check if the current front intake roller speed is within tolerance of the desired speed.
   * @param desiredSpeed The front intake roller speed to check, in rotations per second.
   * @return Whether the front intake roller is at the desired speed.
   * True or false.
   */
  public boolean getFrontIntakeUpToSpeed(double desiredSpeed) {
    if (Constants.kPrintSubsystemFrontIntake){System.out.println("ShooterSubsystem: getShooterUpToSpeed");}
    return ((this.getFrontIntakeSpeed() >= (desiredSpeed - Constants.kShooterSpeedTolerance)) && (this.getFrontIntakeSpeed() <= (desiredSpeed + Constants.kShooterSpeedTolerance)));
  }

  /**
   * Check if the current front intake roller speed is within tolerance of the desired speed.
   * This is an overload.
   * If the desired speed is not passed to this method, it will use the last speed set with {@link #setFrontIntakeSpeed}.
   * @return Whether the front intake roller is at the desired speed.
   */
  public boolean getFrontIntakeUpToSpeed() {
    return this.getFrontIntakeUpToSpeed(m_desiredSpeed);
  }

  /**
   * Add all of the motors in the front intake subsystem to the Orchestra.
   * I want the robot to sing.
   * @param robotOrchestra The Orchestra to add the motors as instruments to.
   */
  public void addToOrchestra(Orchestra robotOrchestra) {
    robotOrchestra.addInstrument(m_frontIntakeRotate, 6);
    robotOrchestra.addInstrument(m_frontIntakeSpin, 6);
  }

   /**
   * Sets all motors in the front intake subsystem to play a tone at the requested frequency.
   * @param toneInput A percentage, mapped to kMusicToneTable lookup table.
   */
  public void setMusicToneFrequency(double toneInput) {
    m_frontIntakeRotate.setControl(frontIntakeMusicToneRequest.withAudioFrequency(Constants.kMusicToneTable.get(toneInput)));
    m_frontIntakeSpin.setControl(frontIntakeMusicToneRequest.withAudioFrequency(Constants.kMusicToneTable.get(toneInput)));
  }  

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("FrontIntakeSubsystem");
    builder.addDoubleProperty("Front Intake Speed", this::getFrontIntakeSpeed, null);
    builder.addDoubleProperty("Front Intake Position", this::getFrontIntakePosition, null);
    builder.addBooleanProperty("Front Intake in Position?", this::getFrontIntakeInPosition, null);
    builder.addBooleanProperty("Front Intake up to Speed?", this::getFrontIntakeUpToSpeed, null);
    builder.addDoubleProperty("Front Intake Desired Position", () -> {return m_desiredPosition;}, null);
    builder.addDoubleProperty("Front Intake Desired Speed", () -> {return m_desiredSpeed;}, null);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
