package frc.robot;

import java.io.File;
import java.util.ArrayList;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

/* TO DO
 * Add overloads for the constructor:
 * - No arguments
 * 
 * Add getNewSelectionList(): Replace or add an ArrayList to the object.
 * Add parseDirectory(): Creates a list based on a folder path and a file extension. 
 */

/**
 * A selector, that allows you to page though an array for a specific index.
 * Created primarily to select an autonomous from an array of different options, but can be used for other purposes.
 */
public class Selector implements NTSendable{
    
    int currentSelection = 0;
    ArrayList<String> selectionNames = new ArrayList<String>();
    String currentSelectionName = "";

    /**
     * Constructs an instance of the selector.
     * Assumes that the selector list is a list of files.
     * @param folderPath The file directory on the roboRIO that contains the files you want to add to the list.
     * @param extension The extension of the files that you want to add to the list.
     * Other file types in the file directory will be ignored.
     * Include the '.'
     */
    public Selector(File folderPath, String extension) {
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

    /**
     * Constructs an instance of the selector.
     * Assumes that the selector list is a pre-existing list of Strings.
     * @param preExistingList A pre-defined list.
     */
    public Selector(ArrayList<String> preExistingList) {
        this.selectionNames = preExistingList;
    }

    /**
     * Decrements the current index of the list by 1.
     * The index cannot be less than 0, or greater than the size of the list.
     */
    public void decrementSelection() {
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

    /**
     * Increments the current index of the list by 1.
     * The index cannot be less than 0, or greater than the size of the list.
     */
    public void incrementSelection() {
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

    /**
     * Prints the contents of the list to the terminal.
     */
    public void printSelections() {
        System.out.println(this.selectionNames.toString());
    }

    /**
     * Get the String at the current index in the list.
     * @return The current selection, as a String.
     */
    public String getCurrentSelectionName() {
        return this.currentSelectionName;
    }
    
    /**
     * Sets the current selection to the String at the current index in the list.
     */
    public void setCurrentSelectionName() {
        this.currentSelectionName = this.selectionNames.get(this.currentSelection);
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("Selector");
        builder.addStringProperty("Current Selection", this::getCurrentSelectionName, null); 
    }
}
