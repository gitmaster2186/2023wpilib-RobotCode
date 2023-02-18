package frc.robot;

public class ConstantsConfiguration {
    private static ConstantsConfiguration configuredConstants = null;
    public static final double kP = Constants.kP;
    public static final double kI = Constants.kI;
    public static final double kD = Constants.kD;
    
    public static final double DRIVETRAIN_WHEELBASE_METERS = Constants.DRIVETRAIN_WHEELBASE_METERS; 

    public static final int DRIVETRAIN_PIGEON_ID = Constants.DRIVETRAIN_PIGEON_ID; 
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = Constants.BACK_LEFT_MODULE_STEER_MOTOR; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER= Constants.FRONT_LEFT_MODULE_STEER_MOTOR; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Constants.FRONT_LEFT_MODULE_STEER_OFFSET; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = Constants.FRONT_RIGHT_MODULE_STEER_MOTOR; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = Constants.FRONT_RIGHT_MODULE_STEER_ENCODER; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = Constants.FRONT_RIGHT_MODULE_STEER_ENCODER; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Constants.FRONT_RIGHT_MODULE_STEER_OFFSET; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = Constants.BACK_LEFT_MODULE_DRIVE_MOTOR; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = Constants.BACK_LEFT_MODULE_STEER_MOTOR; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = Constants.BACK_LEFT_MODULE_STEER_ENCODER; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Constants.BACK_LEFT_MODULE_STEER_OFFSET; 


    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = Constants.BACK_RIGHT_MODULE_STEER_MOTOR; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = Constants.BACK_RIGHT_MODULE_STEER_ENCODER; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Constants.BACK_RIGHT_MODULE_STEER_OFFSET; 

      // Prabhu - Max voltage changed from 12 to 2
    public static double MAX_Voltage= Constants.MAX_Voltage;
    
    public static ConstantsConfiguration getInstance()
    {
        if (configuredConstants == null){
            configuredConstants = new ConstantsConfiguration();
        }
  
        return configuredConstants;
    }

     

}
