package teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.LCamConeLocDetection;

public class resources_NanoTrojans1 {
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
    public Servo clawLift = null;
    public Servo armLift = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;
    public DcMotor lsRight = null;
    public DcMotor lsLeft = null;
    //public DcMotor intake = null;
    public CRServo planeLaunch = null;
    //public CRServo robotLift = null;

    //public DcMotor dcArm;
    public int frontLeftMotorCounts = 0;


    //The following are for single camera
    public int frontRightMotorCounts = 0;
    public int rearLeftMotorCounts = 0;
    public int rearRightMotorCounts = 0;
    public controls_NanoTrojans g2control;

    //private BNO055IMU imu;

    public DistanceSensor leftClawDistanceSensor;
    public DistanceSensor rightClawDistanceSensor;
    public ColorSensor rightClawColorSensor;
    public ColorSensor leftClawColorSensor;

    private HuskyLens huskyLens;
public resources_NanoTrojans1(HardwareMap hardwareMap){
//    frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
//    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//    backRight = hardwareMap.get(DcMotor.class, "backRight");
    lsRight = hardwareMap.dcMotor.get("lsRight");
    lsLeft = hardwareMap.dcMotor.get("lsLeft");
    //intake = hardwareMap.dcMotor.get("intake");

    //Servo Motors
    planeLaunch = hardwareMap.crservo.get("planeLaunch");
    //robotLift = hardwareMap.crservo.get("robotLift");

    //hang


    // get 2 claw motors
    clawLeft = hardwareMap.servo.get("clawLeft");
    clawRight = hardwareMap.servo.get("clawRight");

    // get 2 arm motors
    clawLift = hardwareMap.servo.get("clawLift");
    armLift = hardwareMap.servo.get("armLift");

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

    rightClawColorSensor = hardwareMap.colorSensor.get("rightclawcolor");
    leftClawColorSensor = hardwareMap.colorSensor.get("leftclawcolor");
}
}
