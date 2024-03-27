// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// The shooter intake subsystem is the portion of the shooter that is NOT responsible for ejecting notes at high velocities.
public class ShooterIntakeSubsystem extends SubsystemBase {

  //Open Motors
  TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");
  TalonFX m_ShooterIntakeRotate = new TalonFX(Constants.kShooterIntakeRotateCanID, "Canivore");

  private final VelocityVoltage ShooterIntakeVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private final PositionVoltage ShooterIntakeMoveRequest = new PositionVoltage(0).withSlot(0);

  private final DutyCycleOut ShooterIntakeRotateVoltageRequest = new DutyCycleOut(0, false, false, false, false);

  private final PositionVoltage ShooterIntakePosition = new PositionVoltage(0).withSlot(1);


  // Constructs an instance of the shooter intake subsystem.

  public ShooterIntakeSubsystem() {
  //=======================================shouldn't we be doing something with velocity voltage motor here to set it up?
  this.configureShooterIntakeSpin(m_ShooterIntakeSpin);
  this.configureShooterIntakeRotate(m_ShooterIntakeRotate);
  }
  
  // Start of sensor related methods
  /**
   * Check if a note is in front of the intake beam break.
   * @return Whether the intake beam break detects a note.
   * True or false.
   */
  
  /**
   * Check if a note is in front of the shooter beam break.
   * @return Whether the shooter beam break detects a note.
   * True or false.
   */
  // End of sensor related methods

  /**
   * Check if a note is in front of either the intake beam break or the shooter beam break.
   * @return Whether either beam break detects a note.
   * True or false.
   */

  /**
   * Sets the speed of the shooter intake motor.
   * @param speed The desired speed of the shooter intake, in rotations per second.
   */
  public void setShooterIntakeSpeed(double speed) {
    if (Constants.kMotorEnableShooterIntakeSpin == 1){
      //System.out.println("ShooterIntakeSubsystem: setShooterIntakeSpeed");
      m_ShooterIntakeSpin.setControl(ShooterIntakeVelocity.withVelocity(speed));
    }
  }
/*
  public void setShooterIntakeSpinPosition(double position){
    m_ShooterIntakeSpin.setPosition(0);
      if (Constants.kMotorEnableShooterIntakeSpin == 1){
      //System.out.println("ShooterIntakeSubsystem: setShooterIntakeSpeed");
      m_ShooterIntakeSpin.setControl(ShooterIntakePosition.withPosition(position));
    }
  }

  public boolean getShooterIntakeSpinInPosition(double position){
    if (Math.abs(position -  Constants.kShooterIntakeSpinPositionNote) < Constants.kShooterIntakeSpinPositionTolerance){
      return true;
    } else {
      return false;
    }
  }
*/

  public void setShooterIntakeRotate(double rotations){
    if (Constants.kMotorEnableShooterIntakeRotate ==1) {
      System.out.println("ShooterIntakeSubsystem: " + rotations);
     // m_ShooterIntakeRotate.setControl(ShooterIntakeMoveRequest.withPosition(rotations));
    }

  }

  public boolean getShooterIntakeInPosition(double rotations){

    if ((Math.abs(m_ShooterIntakeRotate.getPosition().getValueAsDouble()) < Constants.kShooterIntakeTrapRotations + 10)){
      return true;
    } else {
      return false;
    }

  }

  public double getShooterIntakePosition() {
    return m_ShooterIntakeRotate.getPosition().getValueAsDouble();
  }

  public void setShooterIntakeRotateZeroOutput() {
    m_ShooterIntakeRotate.setControl(ShooterIntakeRotateVoltageRequest);
  }

  //Leaving this comment as the start of a to-do list.
  //asdfasfadsadsfda To Do here

  public void SetShooterIntakeRotateNeutralMode(NeutralModeValue NeutralMode) {
    var shooterIntakeNeuMotOut = new MotorOutputConfigs();
    var shooterIntakeCurrentConfigurator = m_ShooterIntakeRotate.getConfigurator();

    shooterIntakeCurrentConfigurator.refresh(shooterIntakeNeuMotOut);
    shooterIntakeNeuMotOut.NeutralMode = NeutralMode;
    shooterIntakeCurrentConfigurator.apply(shooterIntakeNeuMotOut);
  }

  public void configureShooterIntakeSpin(TalonFX shooterIntakeSpin) {
    TalonFXConfiguration shooterIntakeSpinVelocityConfig = new TalonFXConfiguration();
    shooterIntakeSpinVelocityConfig.Slot0.kP = Constants.kShooterIntakeSpinProportional; // An error of 1 rotation per second results in 2V output
    shooterIntakeSpinVelocityConfig.Slot0.kI = Constants.kShooterIntakeSpinIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    shooterIntakeSpinVelocityConfig.Slot0.kD = Constants.kShooterIntakeSpinDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    shooterIntakeSpinVelocityConfig.Slot0.kV = Constants.kShooterIntakeSpinVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    shooterIntakeSpinVelocityConfig.Voltage.PeakForwardVoltage = Constants.kShooterIntakeSpinMaxForwardVoltage;
    shooterIntakeSpinVelocityConfig.Voltage.PeakReverseVoltage = Constants.kShooterIntakeSpinMaxReverseVoltage;
    shooterIntakeSpinVelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kShooterIntakeSpinSupplyCurrentLimit;
    shooterIntakeSpinVelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kShooterIntakeSpinVoltageClosedLoopRampPeriod;
    shooterIntakeSpinVelocityConfig.MotorOutput.Inverted = Constants.kShooterIntakeSpinDirection;
    shooterIntakeSpinVelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    //Setting the config option that allows playing music on the motor during disabled.
    shooterIntakeSpinVelocityConfig.Audio.AllowMusicDurDisable = true;

    StatusCode shooterIntakeSpinStatus = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      shooterIntakeSpinStatus = shooterIntakeSpin.getConfigurator().apply(shooterIntakeSpinVelocityConfig);
      if (shooterIntakeSpinStatus.isOK()) break;
    }
    if (!shooterIntakeSpinStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + shooterIntakeSpinStatus.toString());
    }
  }

  public void configureShooterIntakeRotate(TalonFX shooterIntakeRotate) {
    TalonFXConfiguration shooterIntakeRotateConfig = new TalonFXConfiguration();
    shooterIntakeRotateConfig.Slot0.kP = Constants.kShooterIntakeRotateProportional; // An error of 1 rotation per second results in 2V output
    shooterIntakeRotateConfig.Slot0.kI = Constants.kShooterIntakeRotateIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    shooterIntakeRotateConfig.Slot0.kD = Constants.kShooterIntakeRotateDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    shooterIntakeRotateConfig.Slot0.kV = Constants.kShooterIntakeRotateFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    shooterIntakeRotateConfig.Voltage.PeakForwardVoltage = Constants.kShooterIntakeRotateMaxForwardVoltage;
    shooterIntakeRotateConfig.Voltage.PeakReverseVoltage = Constants.kShooterIntakeRotateMaxReverseVoltage;
    shooterIntakeRotateConfig.CurrentLimits.SupplyCurrentLimit = Constants.kShooterIntakeRotateSupplyCurrentLimit;
    shooterIntakeRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kShooterIntakeRotateVoltageClosedLoopRampPeriod;
    shooterIntakeRotateConfig.MotorOutput.Inverted = Constants.kShooterIntakeRotateDirection;
    shooterIntakeRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterIntakeRotateConfig.MotionMagic.MotionMagicAcceleration = Constants.kShooterArmRotateMotionMagicAcceleration;
    shooterIntakeRotateConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.kShooterArmRotateMotionMagicCruiseVelocity;
        
    //Setting the config option that allows playing music on the motor during disabled.
    shooterIntakeRotateConfig.Audio.AllowMusicDurDisable = true;

    StatusCode shooterIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      shooterIntakeRotateStatus = shooterIntakeRotate.getConfigurator().apply(shooterIntakeRotateConfig);
      if (shooterIntakeRotateStatus.isOK()) break;
    }
    if (!shooterIntakeRotateStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + shooterIntakeRotateStatus.toString());
    }
  }

  /**
   * Add all of the motors in the shooter intake subsystem to the Orchestra.
   * I want the robot to sing.
   * @param robotOrchestra The Orchestra to add the motors as instruments to.
   */
  public void addToOrchestra(Orchestra robotOrchestra) {
    robotOrchestra.addInstrument(m_ShooterIntakeSpin);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    
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
