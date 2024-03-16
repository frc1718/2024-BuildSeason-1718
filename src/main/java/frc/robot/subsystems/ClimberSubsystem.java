// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The climber subsystem utilizies cutting edge <i>Pi-Climb®</i> technology to lift the robot off the ground.
 */
public class ClimberSubsystem extends SubsystemBase {

  boolean m_preClimbActuated = false;
  String m_climbLocation;
  double m_desiredPosition = 0;

  //RightClimb Should be a follower of leftclimb
  TalonFX m_LeftClimb = new TalonFX(Constants.kLeftClimbCanID, "Canivore");
  TalonFX m_RightClimb = new TalonFX(Constants.kRightClimbCanID, "Canivore");

  //private final MotionMagicVoltage climberMove = new MotionMagicVoltage(0.0, true, 0, 0, false, false, false);
  private final PositionVoltage climberMoveRequest = new PositionVoltage(0).withSlot(0);
  /**
   * Constructs an instance of the climber subsystem.
   * The motor configuration for the climber is done here.
   */
  public ClimberSubsystem() {
    this.configureLeftClimb(m_LeftClimb);
    this.configureRightClimb(m_RightClimb);
  }

  /**
   * Configures Climber Motors
   * 
   */
  public void configureLeftClimb(TalonFX leftClimb){
 
    //Start Configuring Climbers
    TalonFXConfiguration leftClimberConfig = new TalonFXConfiguration();

    leftClimberConfig.MotorOutput.Inverted = Constants.kLeftClimberDirection;
    leftClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftClimberConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftClimberSupplyCurrentLimit;
    leftClimberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftClimberVoltageClosedLoopRampPeriod;
    leftClimberConfig.Voltage.PeakForwardVoltage = Constants.kLeftClimberMaxForwardVoltage;
    leftClimberConfig.Voltage.PeakReverseVoltage = Constants.kLeftClimberMaxReverseVoltage;
    leftClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0 = leftClimberConfig.Slot0;
    slot0.kP = Constants.kLeftClimberProportional;
    slot0.kI = Constants.kLeftClimberIntegral;
    slot0.kD = Constants.kLeftClimberDerivative;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kLeftClimberVelocityFeedFoward;
    //slot0.kS = Constants.kClimberStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    //Setting the config option that allows playing music on the motor during disabled.
    leftClimberConfig.Audio.AllowMusicDurDisable = true;
 
    StatusCode climberStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climberStatus = leftClimb.getConfigurator().apply(leftClimberConfig);
      if (climberStatus.isOK()) break;
    }
    if (!climberStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climberStatus.toString());
    }
    m_LeftClimb.setPosition(0);
  }

  public void configureRightClimb(TalonFX rightClimb){
    //Start Configuring Climbers
    TalonFXConfiguration rightClimberConfig = new TalonFXConfiguration();

    rightClimberConfig.MotorOutput.Inverted = Constants.kRightClimberDirection;
    rightClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightClimberConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightClimberSupplyCurrentLimit;
    rightClimberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightClimberVoltageClosedLoopRampPeriod;
    rightClimberConfig.Voltage.PeakForwardVoltage = Constants.kRightClimberMaxForwardVoltage;
    rightClimberConfig.Voltage.PeakReverseVoltage = Constants.kRightClimberMaxReverseVoltage;
    rightClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0 = rightClimberConfig.Slot0;
    slot0.kP = Constants.kRightClimberProportional;
    slot0.kI = Constants.kRightClimberIntegral;
    slot0.kD = Constants.kRightClimberDerivative;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kRightClimberVelocityFeedFoward;
    //slot0.kS = Constants.kClimberStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving
 
    StatusCode climberStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climberStatus = rightClimb.getConfigurator().apply(rightClimberConfig);
      if (climberStatus.isOK()) break;
    }
    if (!climberStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climberStatus.toString());
    }
    m_RightClimb.setPosition(0);
  }
    

  /**
   * Sets the position to move the climber to.  
   * @param desiredPosition Desired climber position, in rotations.
   */
  public void setClimberDesiredPosition(double desiredPosition) {
    if (Constants.kMotorEnableClimber == 1) {
      // System.out.println("Subsystem: Climber - setClimberDesiredPosition");
      m_LeftClimb.setControl(climberMoveRequest.withPosition(desiredPosition));
      m_RightClimb.setControl(climberMoveRequest.withPosition(desiredPosition));
    }
      m_desiredPosition = desiredPosition;
  }

  /**
   * Sets whether the climber is in the pre-climb position.
   * @param preClimbActuated If the climber is in the pre-climb position.
   * True or false.
   */
  public void setPreClimbActuated(boolean preClimbActuated){
    m_preClimbActuated = preClimbActuated;
  }

  public void setClimbLocation(String climbLocation) {
    m_climbLocation = climbLocation;
  }
  // m_climbLocation should be either "RightClimb", "LeftClimb", or "FarClimb"
  public String getClimbLocation() {
    return m_climbLocation;
  }
  /**
   * Checks if the climber is in the pre-climb position.
   * @return Whether the climber is in the pre-climb position.
   * True or false.
   */
  public boolean getPreClimbActuated(){
    //System.out.println("Subsystem: Climber - getPreClimbActuated");
    return m_preClimbActuated;
  }

  public boolean getPreClimbNotActuated(){
    //System.out.println("Subsystem: Climber - getPreClimbActuated");
    return !m_preClimbActuated;
  }

  /**
   * Get the current climber position.
   * @return The climber motor position, in rotations.
   */
  public double getClimberPosition() {   
    //System.out.println("Subsystem: Climber - getClimberPosition");
    return m_LeftClimb.getPosition().getValueAsDouble();
  }

  /**
   * Check if the current climber position is at the desired position, within the constant climber tolerance.
   * @param desiredPosition The climber position to check, in rotations.
   * @return Whether the climber is at the desired position.
   * True or false.
   */
  public boolean getClimberInPosition (double desiredPosition) {
    //System.out.println("Subsystem: Climber - getClimberInPosition");
    return ((this.getClimberPosition() > (desiredPosition - Constants.kClimberTolerancePos)) && (this.getClimberPosition() < (desiredPosition + Constants.kClimberTolerancePos)));

  }

  /**
   * Check if the current climber position is at the desired position, within the constant climber tolerance.
   * This is an overload.
   * If the desired position is not passed to this method, it will use the last position set with {@link #setClimberDesiredPosition}.
   * @return Whether the climber is at the desired position.
   */
  public boolean getClimberInPosition() {
    return ((this.getClimberPosition() > (m_desiredPosition - Constants.kClimberTolerancePos)) && (this.getClimberPosition() < (m_desiredPosition + Constants.kClimberTolerancePos)));
  }

  public void SetLeftClimberNeutralMode(NeutralModeValue NeutralMode) {
    var neuMotOut = new MotorOutputConfigs();
    var currentConfigurator = m_LeftClimb.getConfigurator();
    currentConfigurator.refresh(neuMotOut);
    neuMotOut.NeutralMode = NeutralMode;
    currentConfigurator.apply(neuMotOut);
  }

  public void SetRightClimberNeutralMode(NeutralModeValue NeutralMode) {
    var neuMotOut = new MotorOutputConfigs();
    var currentConfigurator = m_RightClimb.getConfigurator();
    currentConfigurator.refresh(neuMotOut);
    neuMotOut.NeutralMode = NeutralMode;
    currentConfigurator.apply(neuMotOut);
  }

  /**
   * Add all of the motors in the climber subsystem to the Orchestra.
   * I want the robot to sing.
   * @param robotOrchestra The Orchestra to add the motors as instruments to.
   */
  public void addToOrchestra(Orchestra robotOrchestra) {
    robotOrchestra.addInstrument(m_LeftClimb);
    robotOrchestra.addInstrument(m_RightClimb);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ClimberSubsystem");
    builder.addDoubleProperty("Climber Position", this::getClimberPosition, null);
    builder.addBooleanProperty("Climber Pre-Actuated?", this::getPreClimbActuated, null);
    builder.addBooleanProperty("Climber in Position?", this::getClimberInPosition, null);  
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
