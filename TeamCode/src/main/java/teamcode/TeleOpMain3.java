//package org.firstinspires.ftc.teamcode;
package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOpMain3", group="TeleOp")
public class TeleOpMain3 extends LinearOpMode {

    private DcMotor intake = null;
    private DcMotor lsRight = null;
    private DcMotor lsLeft = null;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //servo motors
    private Servo planeLaunch = null;

    //2 claws servo motors
    private Servo clawLeft = null;
    private Servo clawRight = null;

    //2 arms servo motors
    private Servo clawLift = null;
    private Servo armLift = null;

    private final double driveAdjuster = 1;

    // the following are for huskylen
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;

    BNO055IMU imu;
    Orientation angles;

    private DriveControl_NanoTorjan driveControl;
    //private DriveControl driveControl;

    @Override
    public void runOpMode()  throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        lsRight = hardwareMap.dcMotor.get("lsRight");
        lsLeft = hardwareMap.dcMotor.get("lsLeft");
        intake = hardwareMap.dcMotor.get("intake");

        //Servo Motors
        planeLaunch = hardwareMap.servo.get("planeLaunch");

        // get 2 claw motors
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight= hardwareMap.servo.get("clawRight");

        // get 2 arm motors
        clawLift = hardwareMap.servo.get("clawLift");
        armLift= hardwareMap.servo.get("armLift");


        // huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);


        rateLimit.expire();

       /* if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        } */


        //huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        //********************** Husky Lens end *********************


        telemetry.update();
        // because the gear is always outside or inside then one size of wheels need to be revers
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);



        driveControl = new DriveControl_NanoTorjan(frontLeft, frontRight, backLeft, backRight, imu);
        //driveControl = new DriveControl(frontLeft, frontRight, backLeft, backRight, imu);


        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double lspower = 0;
        waitForStart();
        //while (!isStopRequested()) {
        while (opModeIsActive()) {

            //Call Robot base movement algorithem to drive the base
            driveControl.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

           // Game Pad 2 controller for other motors
           // control intake motor
           intake.setPower(gamepad2.left_stick_y);
           int currentPosition = intake.getCurrentPosition();
           telemetry.addData("Encoder Position", currentPosition);

            //lift power take from the second game pad
            lspower= gamepad2.right_stick_y*0.2;
            lsRight.setPower(lspower);
            lsLeft.setPower(-lspower);


            /* HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            } */

            //Plane launcher
            //if(gamepad2.left_trigger >=0.1){
            if(gamepad2.y){
                // 1 is the after launch position
                planeLaunch.setPosition(1);
                // after launched wait 2 seconds move back to ready position
                sleep(1500);
                // 0.4 is the ready position
                planeLaunch.setPosition(0.4);
            }

            //Claw contols  -  close
            if(gamepad2.left_trigger >=0.1) {
                clawLeft.setPosition(0.33);
                clawRight.setPosition(0.25);
            }

            //Claw   -   open
            if(gamepad2.right_trigger >=0.1) {
                clawLeft.setPosition(0.10);
                clawRight.setPosition(0.5);
            }

            // Move arm to ready pick up pixel position
            if(gamepad2.left_bumper ) {

                clawLift.setPosition(1);
                sleep(2000);

                clawLift.setPosition(0.5);
                armLift.setPosition(0.5);
                sleep(2000);
                armLift.setPosition(0.3);
                sleep(2000);

                armLift.setPosition(0);
                sleep(2000);

            }

            // Move arm to ready place pixel position
            if(gamepad2.right_bumper ) {
                clawLift.setPosition(0.3);
                sleep(2000);
                clawLift.setPosition(0.5);
                armLift.setPosition(0.8);
                sleep(2000);
                clawLift.setPosition(1);

            }


            telemetry.update();

        }

//        while (opModeIsActive()) {
//
//
//
//        }
    }
}
