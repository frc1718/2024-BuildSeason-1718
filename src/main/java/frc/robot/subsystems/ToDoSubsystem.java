// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Deep Thoughts (by Jack Handy)
    // 1. What can we automate?
    // 2. Do we need a sensor so that we know the climber is set or to zero it?
    // 3. Do we need a manual move of the arm if the cancoder gets ripped or broken?
    // 4. Need a lower limit on the arm once we've released the intake.
    // 5. Intake comes up when we have a note
    // 6. Should we have blinking LEDs when a note is picked up, yeah pretty sure
    // 7. Spin up shooter automatically when the note is intaked, or when we reach a point on the field - how do we watch current draw
    // 7.5 Should we also be active moving the arm based on where the robot is on the field once it has a note
    // 8. When shoot button is released, drop arm back to intake position automatically
    // 9. Need speed control based on arm height
    // 10. Are we shooting based on april tag immediate good check, or are we doing it based on location
    // 11. Location would be better, how quick can we go april tag -> location for a final check
    // 12. Should the shooter lower from the amp position when the driver releases the shooter button
    // 13. Can we auto-align on the amp and the climb
    // 14. Brake when shooting first, then shoot on move?

    // Climb Mode Automation:
    // 1. Operator presses 2 buttons at the same time
    //    Arm lifts to pre climb
    //    Intake actuator pulls in and lets intake rotate down
    // 2. Driver pulls climb button
    //    Is arm on the tower during lift?  Might need rollers
    //    Does arm need to lower as we raise?
    //    Automatically shoot at height

public class ToDoSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ToDoSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command climberMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean climberCondition() {
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
