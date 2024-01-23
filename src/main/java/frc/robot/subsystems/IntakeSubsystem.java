// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}

  // TalonFX m_FrontIntakeRotate = new TalonFX(Constants.kFrontIntakeRotateCanID, "Canivore");
  // TalonFX m_FrontIntakeSpin = new TalonFX(Constants.kFrontIntakeSpinCanID, "Canivore");

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command suck() {
    return run(
        () -> {
          // set.intakeposition()
          /* set.intakeSpeed(suck) */
        });
  }

  public Command intakeDown() {
    return run(
        () -> {
          /* set.intakeposition() */
        });
  }

  public Command intakeUp() {
    return run(
        () -> {
          /* set.intakeposition() */
        });
  }

  public Command spit() {
    return run(
        () -> {
          // set.intakeposition
          /* set.intakeSpeed(spit) */
        });
  }

    public Command home() {
    return runOnce(
        () -> {
          // set.intakeposition
          /* set.intakeSpeed(stop) */
        });
  }

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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
