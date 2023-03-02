package frc.robot.utility_classes;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SemiAbsoluteEncoder {

   private CANSparkMax encoderController;

   private RelativeEncoder relative;
   private SparkMaxAbsoluteEncoder absolute;

   private double conversionFactor = 1;


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

   public RelativeEncoder getSparkMAXEncoder(){
      return relative;
   }

   public void setZeroOffset(double offset){
      absolute.setZeroOffset(offset);
      relative.setPosition(absolute.getPosition()); 
   }
   
   public void setPositionConversionFactor(double multiplyEncoderBy){
      relative.setPositionConversionFactor(multiplyEncoderBy);
   }
}