package frc.robot.subsystems.wrist;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import frc.lib.team5557.factory.SparkMaxFactory.PIDConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SoftLimitsConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants;

public class WristConstants {
    public static final double kGearReduction = 75.0;
    public static final double kFirstSprocketTeethCount = 16.0; //teeth
    public static final double kSecondSprocketTeethCount = 48.0; //teeth
    public static final double kRotationsPerDegree = kGearReduction * (kSecondSprocketTeethCount / kFirstSprocketTeethCount) / 360.0;

    public static final double kEncoderHomePosition = 270.0; //degrees
    public static final double kPadding = 1.0; // degrees
    public static final double kCruiseVelocity = 150.0; // degrees/sec
    public static final double kTimeToCruise = 0.1; // sec

    public static final double kHomeVoltage = 2.0;
    public static final double kHomeAmpsThreshold = 15.0;

    public static final double kMinAngle = 170.0; //degrees
    public static final double kMaxAngle = 270.0; //degrees

    public static final double kWristkP = 0.1;
    public static final double kWristkI = 0.0;
    public static final double kWristkD = 0.0;

    public static final double kWristkS = 0.0;
    public static final double kWristkG = 0.0;
    public static final double kWristkA = 0.0;
    public static final double kWristkV = 0.0;
    
    public static final SoftLimitsConfiguration kLimitConfiguration = new SoftLimitsConfiguration();
    static {
        kLimitConfiguration.kUpperLimit = degreesToRotations(kMaxAngle);
        kLimitConfiguration.kLowerLimit = degreesToRotations(kMinAngle);
    }

    public static final PIDConfiguration kPIDConfiguration = new PIDConfiguration();
    static {
        kPIDConfiguration.kP = kWristkP;
        kPIDConfiguration.kI = kWristkI;
        kPIDConfiguration.kD = kWristkD;
        kPIDConfiguration.kF = 0.0;
        kPIDConfiguration.kTolerance = degreesToRotations(kPadding);
    }

    public static final SparkMaxConfiguration kMasterMotorConfiguration = new SparkMaxConfiguration();
    static {
        kMasterMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, Constants.RobotMap.kWristMotor);

        kMasterMotorConfiguration.pid = kPIDConfiguration;
        kMasterMotorConfiguration.limits = kLimitConfiguration;

        kMasterMotorConfiguration.kVoltageCompensation = 12.0;
        kMasterMotorConfiguration.kShouldInvert = true;
        kMasterMotorConfiguration.kIdleMode = IdleMode.kBrake;
        kMasterMotorConfiguration.kOpenLoopRampRate = 1.0;
        kMasterMotorConfiguration.kClosedLoopRampRate = 0.5;
        kMasterMotorConfiguration.kSmartCurrentLimit = 30.0;
    }

    public static double rotationsToDegrees(double rotations) {
        return rotations / kRotationsPerDegree;
    }

    public static double degreesToRotations(double degrees) {
        return degrees * kRotationsPerDegree;
    }

    public static double constrainDegrees(double degrees) {
        return MathUtil.clamp(degrees, kMinAngle, kMaxAngle);
    }
}