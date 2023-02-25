package frc.robot.utility_classes;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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