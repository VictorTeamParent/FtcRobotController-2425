package teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class RobotDefinition {

    public DcMotor lsRight = null;
    public DcMotor lsLeft = null;

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    //servo motors
    public CRServo planeLaunch = null;

    //2 claws servo motors
    public Servo clawLeft = null;
    public Servo clawRight = null;

    //2 arms servo motors
    public Servo clawLift = null;
    public Servo armLift = null;
    public CRServo robotLift = null;

    public DcMotor dcArm;


    public final double driveAdjuster = 1;

    // the following are for huskylen
    public HuskyLens huskyLens;
    public final int READ_PERIOD = 1;

    BNO055IMU imu;
    Orientation angles;

    public DriveControl_NanoTorjan driveControl;
    //public DriveControl driveControl;

    public controls_NanoTrojans g2control;
    
       
    public RobotDefinition() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        lsRight = hardwareMap.dcMotor.get("lsRight");
        lsLeft = hardwareMap.dcMotor.get("lsLeft");
        //intake = hardwareMap.dcMotor.get("intake");

        //Servo Motors
        planeLaunch = hardwareMap.crservo.get("planeLaunch");
        robotLift = hardwareMap.crservo.get("robotLift");

        //hang


        // get 2 claw motors
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");

        // get 2 arm motors
        clawLift = hardwareMap.servo.get("clawLift");
        armLift = hardwareMap.servo.get("armLift");

        dcArm = hardwareMap.dcMotor.get("dcArm");

        // huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        driveControl = new DriveControl_NanoTorjan(frontLeft, frontRight, backLeft, backRight, imu);
        //driveControl = new DriveControl(frontLeft, frontRight, backLeft, backRight, imu);
        g2control=new controls_NanoTrojans(lsRight, lsLeft, planeLaunch,
                clawLeft, clawRight, clawLift, armLift);
        
    }
}
