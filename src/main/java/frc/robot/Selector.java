package frc.robot;

import java.io.File;
import java.util.ArrayList;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

/* TO DO
 * Add overloads for the constructor:
 * - No arguments
 * - Pre-existing ArrayList
 * 
 * Add getNewSelectionList(): Replace or add an ArrayList to the object.
 * Add parseDirectory(): Creates a list based on a folder path and a file extension. 
 */

public class Selector implements NTSendable{
    
    int currentSelection = 0;
    ArrayList<String> selectionNames = new ArrayList<String>();
    String currentSelectionName = "";

    public Selector(File folderPath, String extension){
        for (var i : folderPath.listFiles()){
            if(i.isFile() && i.getName().endsWith(extension)){
                //There may be an easier way to strip the extension from the file name.
                this.selectionNames.add(i.getName().substring(0, i.getName().lastIndexOf('.')));
            }
        }

        if (this.selectionNames.size() > 0){
            //Set the current selection name to index 0, if the array isn't empty.
            this.setCurrentSelectionName();
        }
    }

    public void decrementSelection(){
        if (this.currentSelection > 0){
            //If the current auton is not at zero, decrement the number.
            this.currentSelection--;
        }
        else if (this.currentSelection < 0){
            //This shouldn't happen, but fix it if it did.
            this.currentSelection = 0;
        }
        this.setCurrentSelectionName();
    }

    public void incrementSelection(){
        if (this.currentSelection < this.selectionNames.size() - 1){
            //If the current auton is not at the maximum, increment the number.
            this.currentSelection++;
        }
        else if (this.currentSelection > this.selectionNames.size() - 1){
            //This shouldn't happen, but fix it if it did.
            this.currentSelection = this.selectionNames.size() - 1;
        }
        this.setCurrentSelectionName();
    }

    public void printSelections(){
        System.out.println(this.selectionNames.toString());
    }

    public String getCurrentSelectionName(){
        return this.currentSelectionName;
    }
    
    public void setCurrentSelectionName(){
        this.currentSelectionName = this.selectionNames.get(this.currentSelection);
    }

    @Override
    public void initSendable(NTSendableBuilder builder){
        builder.setSmartDashboardType("Selector");
        builder.addStringProperty("Current Selection", () -> {return getCurrentSelectionName();}, null); 
    }
}
