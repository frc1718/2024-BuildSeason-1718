// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The front intake subsystem sucks up notes from the floor and into the robot.
 * <p><i>Nom-nom-nom.</i>
 */
public class FrontIntakeSubsystem extends SubsystemBase {
  /*

  TalonFX m_frontIntakeRotate = new TalonFX(Constants.kFrontIntakeRotateCanID, "Canivore");
  TalonFX m_frontIntakeSpin = new TalonFX(Constants.kFrontIntakeSpinCanID, "Canivore");

  private final MotionMagicVoltage frontIntakeRotation = new MotionMagicVoltage(0.0, true, 0, 0, false, false, false);
  private final VelocityVoltage frontIntakeVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  CANcoder frontIntakeRotateCANcoder = new CANcoder(Constants.kFrontIntakeRotateCancoderCanID);

  */

  Servo frontIntakeServo = new Servo(0);

  public int m_desiredPosition = 0;
  public int m_desiredSpeed = 0;

  /**
   * Constructs an instance of the fron intake subsystem.
   * The motor and sensor configuration is done here.
   */
  public FrontIntakeSubsystem() {
    // Start Configuring CANcoder

    /*

    CANcoderConfiguration frontIntakeRotateCANcoderConfig = new CANcoderConfiguration();
    frontIntakeRotateCANcoderConfig.MagnetSensor.MagnetOffset = 0.0;
    frontIntakeRotateCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    frontIntakeRotateCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // End Configuration of CANcoder
    
    // Start Configuring FrontIntakeVelocity
    TalonFXConfiguration frontIntakeVelocityConfig = new TalonFXConfiguration();

    frontIntakeVelocityConfig.Slot0.kP = Constants.kFrontIntakeSpinProportional; // An error of 1 rotation per second results in 2V output
    frontIntakeVelocityConfig.Slot0.kI = Constants.kFrontIntakeSpinIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    frontIntakeVelocityConfig.Slot0.kD = Constants.kFrontIntakeSpinDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    frontIntakeVelocityConfig.Slot0.kV = Constants.kFrontIntakeSpinVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    frontIntakeVelocityConfig.Voltage.PeakForwardVoltage = Constants.kFrontIntakeSpinMaxForwardVoltage;
    frontIntakeVelocityConfig.Voltage.PeakReverseVoltage = Constants.kFrontIntakeSpinMaxReverseVoltage;

    frontIntakeVelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kFrontIntakeSpinSupplyCurrentLimit;
    frontIntakeVelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kFrontIntakeSpinVoltageClosedLoopRampPeriod;

    StatusCode frontIntakeSpinStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      frontIntakeSpinStatus = m_frontIntakeSpin.getConfigurator().apply(frontIntakeVelocityConfig);
      if (frontIntakeSpinStatus.isOK()) break;
    }
    if (!frontIntakeSpinStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + frontIntakeSpinStatus.toString());
    }
    // End Configuration of FrontIntakeVelocity

    //Start Configuring FrontIntakeRotate
    TalonFXConfiguration frontIntakeRotateConfig = new TalonFXConfiguration();
    MotionMagicConfigs frontIntakeRotateMotionMagic = frontIntakeRotateConfig.MotionMagic;
    frontIntakeRotateMotionMagic.MotionMagicCruiseVelocity = Constants.kFrontIntakeRotateMotionMagicCruiseVelocity; // 5 rotations per second cruise if this is 5
    frontIntakeRotateMotionMagic.MotionMagicAcceleration = Constants.kFrontIntakeRotateMotionMagicAcceleration; // Take approximately 0.5 seconds to reach max vel if this is 10
    // Take approximately 0.2 seconds to reach max accel 
    frontIntakeRotateMotionMagic.MotionMagicJerk = Constants.kFrontIntakeRotateMotionMagicJerk;

    frontIntakeRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kFrontIntakeRotateSupplyCurrentLimit;
    frontIntakeRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kFrontIntakeRotateVoltageClosedLoopRampPeriod;

    Slot0Configs slot0 = frontIntakeRotateConfig.Slot0;
    slot0.kP = Constants.kFrontIntakeRotateProportional;
    slot0.kI = Constants.kFrontIntakeRotateIntegral;
    slot0.kD = Constants.kFrontIntakeRotateDerivative;
    slot0.kV = Constants.kFrontIntakeRotateVelocityFeedFoward;
    slot0.kS = Constants.kFrontIntakeRotateStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    frontIntakeRotateConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    frontIntakeRotateConfig.Feedback.FeedbackRemoteSensorID = frontIntakeRotateCANcoder.getDeviceID();

    StatusCode frontIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      frontIntakeRotateStatus = m_frontIntakeRotate.getConfigurator().apply(frontIntakeRotateConfig);
      if (frontIntakeRotateStatus.isOK()) break;
    }
    if (!frontIntakeRotateStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + frontIntakeRotateStatus.toString());
    }
    //End Configuration

  */
  }

  /**
   * Sets the position of the front intake.
   * @param position The desired position of the front intake, in rotations.
   */
  public void setFrontIntakePosition(int position) {
    //m_frontIntakeRotate.setControl(frontIntakeRotation.withPosition(position));
    System.out.println("FrontIntakeSubsystem - setFrontIntakePosition");
    m_desiredPosition = position;
  }

  /**
   * Sets the speed of the front intake roller.
   * @param speed The desired speed of the front intake roller, in rotations per second.
   */
  public void setFrontIntakeSpeed(int speed) {
    //m_frontIntakeSpin.setControl(frontIntakeVelocity.withVelocity(speed));
    System.out.println("FrontIntakeSubsystem - setFrontIntakeSpeed");
    m_desiredSpeed = speed;
  }

  /**
   * Get the current speed of the front intake roller.
   * @return The speed of the front intake roller motor, in rotations per second.
   */
  public double getFrontIntakeSpeed() {
    System.out.println("FrontIntakeSubsystem: getFrontIntakeSpeed");
    // this WAS the return value.  Replace the "1" m_frontIntakeSpin.getVelocity().getValueAsDouble()
    return 1 ;
  }

  /**
   * Get the current position of the front intake.
   * @return The current position of the front intake, in rotations.
   */
  public double getFrontIntakePosition() {
    System.out.println("FrontIntakeSubsystem: getFrontIntakePosition");
    // this WAS the return value.  Replace the "1" m_frontIntakeRotate.getPosition().getValueAsDouble()
    return 1;
  }
  
  /**
   * Check if the current front intake position is within tolerance of the desired position.
   * @param desiredPosition The front intake position to check, in rotations.
   * @return Whether the shooter arm is at the desired position.
   * True or false.
   */
  public Boolean getFrontIntakeInPosition(int desiredPosition) {
    //Check that the front intake is within the tolerance of the desired position.
    System.out.println("FrontIntakeSubsystem - getFrontIntakeInPosition");
    //return ((this.getFrontIntakePosition() > (desiredPosition - Constants.kFrontIntakeTolerancePos)) && (this.getFrontIntakePosition() < (desiredPosition + Constants.kFrontIntakeTolerancePos)));
    //This return should be removed when the code above is commented out.
    return false;
  }

  /**
   * Check if the current front intake position is within tolerance of the desired position.
   * This is an overload.
   * If the desired position is not passed to this method, it will use the last position set with {@link #setFrontIntakePosition}.
   * @return Whether the front intake is at the desired position.
   * True or false.
   */
  public Boolean getFrontIntakeInPosition() {
    //return this.getFrontIntakeInPosition(m_desiredPosition);
    return false;
  }

  /**
   * Set the position of the front intake servo.
   * The position is a percentage of the full range of the servo.
   * @param servoPos The desired position of the front intake servo.
   * From 0.0 to 1.0.
   */
  public void setServoPosition(double servoPos) {
    frontIntakeServo.set(servoPos);
  }

  /**
   * Get the current position of the front intake servo.
   * @return The current position, as a percentage of the full range of the servo.
   * From 0.0 to 1.0.
   */
  public double getServoPosition() {
    return frontIntakeServo.getPosition();
  }

  /**
   * Check if the current front intake roller speed is within tolerance of the desired speed.
   * @param desiredSpeed The front intake roller speed to check, in rotations per second.
   * @return Whether the front intake roller is at the desired speed.
   * True or false.
   */
  public boolean getFrontIntakeUpToSpeed(int desiredSpeed) {
    System.out.println("ShooterSubsystem: getShooterUpToSpeed");
    return ((this.getFrontIntakeSpeed() >= (desiredSpeed - Constants.kShooterSpeedTolerance)) && (this.getFrontIntakeSpeed() <= (desiredSpeed + Constants.kShooterSpeedTolerance)));
  }

  /**
   * Check if the current front intake roller speed is within tolerance of the desired speed.
   * This is an overload.
   * If the desired speed is not passed to this method, it will use the last speed set with {@link #setFrontIntakeSpeed}.
   * @return Whether the front intake roller is at the desired speed.
   */
  public boolean getFrontIntakeUpToSpeed() {
    //return this.getFrontIntakeUpToSpeed(m_desiredSpeed);
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("FrontIntakeSubsystem");
    builder.addDoubleProperty("Front Intake Speed", this::getFrontIntakeSpeed, null);
    builder.addDoubleProperty("Front Intake Position", this::getFrontIntakePosition, null);
    builder.addDoubleProperty("Front Intake Servo Position", this::getServoPosition, null);
    builder.addBooleanProperty("Front Intake in Position?", this::getFrontIntakeInPosition, null);
    builder.addBooleanProperty("Front Intake up to Speed?", this::getFrontIntakeUpToSpeed, null);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
