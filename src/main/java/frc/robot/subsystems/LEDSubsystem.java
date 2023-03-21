package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;

public class LEDSubsystem extends SubsystemBase {

    public JSONObject currentPattern;

    /**&& A {@link HashMap} that maps the file name of a pattern to a {@JSONObject} */
    HashMap < String, JSONObject > patternMap = new HashMap < String, JSONObject > ();
    
    int time = 0;
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LEDSubsystem() {

        //&& TODO: check the PWM port value, because I'm not sure if 0 is accurate.
        m_led = new AddressableLED(0);

        //&& The default length of our LED strips is 60
        //&& Length is expensive to set, so just set it once and then update data from there
        //&& TODO: change the buffer value to match the length of the LEDs we're actually using
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();


        // H! Get the file for the directory holding the patterns
        File patternDirectory = new File("LEDPatterns");
        // H! Get all the files in said directory
        File[] patterns = patternDirectory.listFiles();

        // H! Initialize the lists of json strings and filenames we'll convert into a map later
        ArrayList<String> jsonStrings = new ArrayList<String>();
        ArrayList<String> validFileNamesFound = new ArrayList<String>();

        try {
        for (File patternFile : patterns) {
            // H! Only add the file if it's .json
            if (patternFile.getName().length() >= 5) {
            if (patternFile.getName().substring(patternFile.getName().length() - 5) == ".json") {
                // H! Read the file and add it to jsonStrings
                jsonStrings.add(Files.readString(patternFile.toPath()));
                // H! Add the filename to validFileNamesFound
                validFileNamesFound.add(patternFile.getName());
            }
            }
        }
        } catch (IOException e) {
        System.out.println("Reading JSON files failed");
        e.printStackTrace();
        }

        // H! Put the list of filenames and list of JSONObjects into a map from one to the other
        for (int i = 0; i < validFileNamesFound.size(); i++) {
        patternMap.put(
            validFileNamesFound.get(i),
            new JSONObject(jsonStrings.get(i))
        );
        }
    }

     public void setPattern(String patternName){
        currentPattern = patternMap.get(patternName);
     }

    public void robotInit() {} 

    public void periodic() {

        JSONArray strandStateIndexes = currentPattern.getJSONArray("strandStateIndexes");
        JSONArray strandStates = currentPattern.getJSONArray("strandStates");

        time ++ ;
        time = time % strandStateIndexes.length();

        int currentStrandStateIndex = (int) strandStateIndexes.get(time);
        strandStateIndexes.length();

        JSONArray strandState = strandStates.getJSONArray(currentStrandStateIndex);


        for (int i = 0; i < m_ledBuffer.getLength(); i++) {

            int loopingI = i % strandState.length();
            final int hue = (strandState.getJSONObject(loopingI)).getInt("H");
            final int saturation = (strandState.getJSONObject(loopingI)).getInt("S");
            final int value = (strandState.getJSONObject(loopingI)).getInt("V");

            m_ledBuffer.setHSV(i, hue, saturation, value);

        }

        m_led.setData(m_ledBuffer);
    }
    
}