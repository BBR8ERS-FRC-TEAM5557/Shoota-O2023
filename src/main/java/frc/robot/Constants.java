package frc.robot;

public class Constants {

    public static boolean kIsReal = Robot.isReal();
    public static boolean kTuningMode = true;

    public class RobotMap {
        public static final int kFLDriveMotor = 25;
        public static final int kFLTurnMotor = 26;
        public static final int kFLCancoder = 13;
        public static final double kFLOffset = -94.538;
        
        public static final int kFRDriveMotor = 23;
        public static final int kFRTurnMotor = 24;
        public static final int kFRCancoder = 12;
        public static final double kFROffset = -221.219;

        public static final int kBLDriveMotor = 27;
        public static final int kBLTurnMotor = 28;
        public static final int kBLCancoder = 14;
        public static final double kBLOffset = -187.357;
        
        public static final int kBRDriveMotor = 21;
        public static final int kBRTurnMotor = 22;
        public static final int kBRCancoder = 11;
        public static final double kBROffset = -39.19;
        


        public static final int kRollerMotor = 41;

        public static final int kWristMotor = 31;


    }
}