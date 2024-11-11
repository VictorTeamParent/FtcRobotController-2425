package teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.LCamConeLocDetection;

public class resources_NanoTrojans {
//    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
//    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
//    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
//    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
//    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch
    OpenCvWebcam webcam2;
    LCamConeLocDetection pipeline2;
    LCamConeLocDetection.LSideConePosition position2 = LCamConeLocDetection.LSideConePosition.OTHER;
//    public DcMotor frontLeft;
//    public DcMotor frontRight;
//    public DcMotor backLeft;
//    public DcMotor backRight;
    public DcMotor lsRight = null;
    public DcMotor lsLeft = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public Servo rintakelift = null;
    public Servo lintakelift = null;

    //servo motors
    public Servo rhsl = null;

    //2 claws servo motors
    public Servo lhsl = null;
    public Servo claw = null;

    //2 arms servo motors
    public CRServo intakewheels = null;

    public Servo clawlift = null;
    public Servo casket = null;

    //public CRServo robotLift = null;

    //public DcMotor dcArm;
    public int frontLeftMotorCounts = 0;


    //The following are for single camera
    public int frontRightMotorCounts = 0;
    public int rearLeftMotorCounts = 0;
    public int rearRightMotorCounts = 0;
    public controls_NanoTrojans1 g2control;

    //private BNO055IMU imu;

    public DistanceSensor leftClawDistanceSensor;
    public DistanceSensor rightClawDistanceSensor;
    public ColorSensor rightClawColorSensor;
    public ColorSensor leftClawColorSensor;

    private HuskyLens huskyLens;
public resources_NanoTrojans(HardwareMap hardwareMap){
//    frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
//    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//    backRight = hardwareMap.get(DcMotor.class, "backRight");
    lsRight = hardwareMap.dcMotor.get("lsRight");
    lsLeft = hardwareMap.dcMotor.get("lsLeft");
    claw = hardwareMap.servo.get("claw");
    intakewheels = hardwareMap.crservo.get("intake");
    clawlift = hardwareMap.servo.get("clawlift");
    rintakelift = hardwareMap.servo.get("rintakelift");
    lintakelift = hardwareMap.servo.get("lintakelift");
    rhsl = hardwareMap.servo.get("rhsl");
    lhsl = hardwareMap.servo.get("lhsl");
    casket = hardwareMap.servo.get("casket");

    //dcArm = hardwareMap.dcMotor.get("dcArm");

    // Set motor directions (adjust as needed based on your robot configuration)
//    frontLeft.setDirection(DcMotor.Direction.FORWARD);
//    frontRight.setDirection(DcMotor.Direction.REVERSE);
//    backLeft.setDirection(DcMotor.Direction.FORWARD);
//    backRight.setDirection(DcMotor.Direction.REVERSE);

    //get imu resource fron control hub
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    parameters.loggingEnabled = true;
//    parameters.loggingTag = "IMU";
//    imu = hardwareMap.get(BNO055IMU.class, "imu");
//    imu.initialize(parameters);

//    rightClawColorSensor = hardwareMap.colorSensor.get("rightclawcolor");
//    leftClawColorSensor = hardwareMap.colorSensor.get("leftclawcolor");
}
}
