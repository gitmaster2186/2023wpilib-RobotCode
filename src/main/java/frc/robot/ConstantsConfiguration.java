package frc.robot;

public class ConstantsConfiguration {
    private static ConstantsConfiguration configuredConstants = null;
    public static final double kP = Constants.kP;
    public static final double kI = Constants.kI;
    public static final double kD = Constants.kD;
    
    public static final double DRIVETRAIN_WHEELBASE_METERS = Constants.DRIVETRAIN_WHEELBASE_METERS; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = Constants.DRIVETRAIN_PIGEON_ID; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = Constants.BACK_LEFT_MODULE_STEER_MOTOR; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER= Constants.FRONT_LEFT_MODULE_STEER_MOTOR; // FIXME Set front left steer encoder ID
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(113.4); // FIXME Measure and set front left steer offset
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Constants.FRONT_LEFT_MODULE_STEER_OFFSET; // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = Constants.FRONT_RIGHT_MODULE_STEER_MOTOR; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = Constants.FRONT_RIGHT_MODULE_STEER_ENCODER; // FIXME Set front right steer encoder ID
    //public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(68.8); // FIXME Measure and set front right steer offset
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Constants.FRONT_RIGHT_MODULE_STEER_OFFSET; // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = Constants.BACK_LEFT_MODULE_DRIVE_MOTOR; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = Constants.BACK_LEFT_MODULE_STEER_MOTOR; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = Constants.BACK_LEFT_MODULE_STEER_ENCODER; // FIXME Set back left steer encoder ID
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(229.6); // FIXME Measure and set back left steer offset
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Constants.BACK_LEFT_MODULE_STEER_OFFSET; // FIXME Measure and set back left steer offset


    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = Constants.BACK_RIGHT_MODULE_STEER_MOTOR; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = Constants.BACK_RIGHT_MODULE_STEER_ENCODER; // FIXME Set back right steer encoder ID
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(207.4); // FIXME Measure and set back right steer offset
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Constants.BACK_RIGHT_MODULE_STEER_OFFSET; // FIXME Measure and set back right steer offset

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
