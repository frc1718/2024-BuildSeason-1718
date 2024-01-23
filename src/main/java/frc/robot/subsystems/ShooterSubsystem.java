// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements NTSendable{
 
  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog); 

  //Open Motors
  // TalonFX m_ShooterRotateLeft = new TalonFX(Constants.kShooterRotateLeftCanID, "Canivore");
  // TalonFX m_ShooterRotateRight = new TalonFX(Constants.kShooterRotateRightCanID, "Canivore");
  // TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");

  public ShooterSubsystem() {
  }

  public Command shoot() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {

        });
  }

  public Command extra() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

        });
  }

  public Command debug(ShooterSubsystem noteSense){
    return runOnce(() -> {System.out.print(noteSense.getNotePresentIntakeString());});
  };
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean shooterCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public boolean getNotePresentIntake() {  
    return (m_BeamBreakIntakeAnalog.getVoltage() >= Constants.kBeamBreakCrossover);
  }
   
  public boolean getNotePresentShooter() {  
    return (m_BeamBreakShooterAnalog.getVoltage() >= Constants.kBeamBreakCrossover);
  }

  public double getVolt() {
    return m_BeamBreakIntakeAnalog.getVoltage();
  }

  public String getNotePresentIntakeString(){
    String result;
    if (m_BeamBreakIntakeAnalog.getVoltage() >= Constants.kBeamBreakCrossover){
       result = "Note Present";
    }
    else{
      result = "Note Not Present";
    }
      return result;
  }

  public void printGetNotePresentShooter() {
    double printStatement = m_BeamBreakShooterAnalog.getVoltage();
    System.out.println(printStatement);
  }

  @Override
  public void initSendable(NTSendableBuilder builder){
    builder.setSmartDashboardType("NoteSensors");
    builder.addDoubleProperty("Current Voltage", () -> {return m_BeamBreakIntakeAnalog.getVoltage();}, null); 
    builder.addBooleanProperty("Beam Broken", () -> {return (m_BeamBreakIntakeAnalog.getVoltage() > Constants.kBeamBreakCrossover);}, null);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
