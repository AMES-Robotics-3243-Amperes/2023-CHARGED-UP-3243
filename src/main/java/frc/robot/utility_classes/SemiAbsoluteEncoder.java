package frc.robot.utility_classes;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.CAN;
import frc.robot.Constants.WristAndArm.MotorIDs;
import frc.robot.subsystems.LegAnkleSubsystem;

public class SemiAbsoluteEncoder {

   private CANSparkMax encoderController;

   private RelativeEncoder relative;
   private SparkMaxAbsoluteEncoder absolute;


   public SemiAbsoluteEncoder(CANSparkMax motorController) {

    encoderController = motorController;

    relative = encoderController.getEncoder();
    absolute = encoderController.getAbsoluteEncoder(Type.kDutyCycle);

    relative.setPosition(absolute.getPosition()); 
    
   }

   /**
    * returns the numbers of rotations
    */
   public double getPosition() {
    
    return relative.getPosition();
   }
   

}