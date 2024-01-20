package frc.robot;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;

/* TO DO
 * Add overloads for the constructor:
 * - No arguments
 * - Pre-existing ArrayList
 * 
 * Add getNewSelectionList(): Replace or add an ArrayList to the object.
 * Add parseDirectory(): Creates a list based on a folder path and a file extension. 
 */

public class NoteSensors implements NTSendable {

    //Open Analog
    AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
    AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog); 

    public NoteSensors() {  
        
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

    @Override
    public void initSendable(NTSendableBuilder builder){
        builder.setSmartDashboardType("NoteSensors");
        builder.addDoubleProperty("Current Voltage", () -> {return m_BeamBreakIntakeAnalog.getVoltage();}, null); 
        builder.addBooleanProperty("Beam Broken", () -> {return (m_BeamBreakIntakeAnalog.getVoltage() > Constants.kBeamBreakCrossover);}, null);
    }
}
