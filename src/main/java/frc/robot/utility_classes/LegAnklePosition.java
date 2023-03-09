// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility_classes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**H! Holds a set of motor positions the robot could go to.
 * 
 */
public class LegAnklePosition {
    public double extension;
    public double pivot;
    public double pitch;
    public double roll; 

    /**H! Creates a new motor position given the extension, pivot, pitch, and roll positions.
     * 
     * @param extension
     * @param pivot
     * @param pitch
     * @param roll
     */
    public LegAnklePosition(double extension, double pivot, double pitch, double roll) {
        this.extension = extension;
        this.pivot = pivot;
        this.pitch = pitch;
        this.roll = roll;
    }

    /**H! Gets the x, y, robot relative pitch, and roll
     * 
     * @return An array of the x, y, robot relative pitch, and roll
     */
    public double[] getKinematicPositions() {
        return new double[] { 
        (extension * Math.cos(Units.rotationsToRadians(pivot)))  +  (Constants.WristAndArm.wristLength * Math.cos(Units.rotationsToRadians(pivot + pitch - 0.5))),
        (extension * Math.sin(Units.rotationsToRadians(pivot)))  +  (Constants.WristAndArm.wristLength * Math.sin(Units.rotationsToRadians(pivot + pitch - 0.5))),
        Units.rotationsToRadians( pivot + pitch - 0.5 ),
        Units.rotationsToRadians( roll )
        };
    }

    /**H! Set the refrences for the pids to this motor position
     * 
     */
    public void applyToMotors(SparkMaxPIDController pidExtension, SparkMaxPIDController pidPivot, SparkMaxPIDController pidPitch, SparkMaxPIDController pidRoll) {
        pidExtension.setReference(extension, CANSparkMax.ControlType.kPosition);
        pidPivot.setReference(pivot, CANSparkMax.ControlType.kPosition);
        pidPitch.setReference(pitch, CANSparkMax.ControlType.kPosition);
        pidRoll.setReference(roll, CANSparkMax.ControlType.kPosition);
    }
}
