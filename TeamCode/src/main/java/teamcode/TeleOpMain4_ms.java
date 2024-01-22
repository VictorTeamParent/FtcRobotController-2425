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
import com.qualcomm.robotcore.hardware.CRServo;

import teamcode.DriveControl_NanoTorjan;

@TeleOp(name = "TeleOpMain4_ms", group = "TeleOp")


public class TeleOpMain4_ms extends LinearOpMode {

    private DcMotor intake = null;
    private DcMotor lsRight = null;
    private DcMotor lsLeft = null;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //servo motors
    private CRServo planeLaunch = null;

    //2 claws servo motors
    private Servo clawLeft = null;
    private Servo clawRight = null;

    //2 arms servo motors
    private Servo clawLift = null;
    private Servo armLift = null;
    private CRServo robotLift = null;




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
        planeLaunch = hardwareMap.crservo.get("planeLaunch");
        robotLift = hardwareMap.crservo.get("robotLift");

        //hang


        // get 2 claw motors
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");

        // get 2 arm motors
        clawLift = hardwareMap.servo.get("clawLift");
        armLift = hardwareMap.servo.get("armLift");


        // huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
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


        Thread baseControlThread = new Thread(new baseControl());
        Thread armControlThread = new Thread(new armControl());

        //Start 2  threads
        baseControlThread.start();
        armControlThread.start();

        // This is the 3rd thread
        //The following  loop is just to keep this main thread running.
        // Add above 2 threads basicall we have 3 threads running
        while (opModeIsActive()) {
              //put some code here for more actions on the control thread
            // Game Pad 2 controller for other motors
            // control intake motor
            intake.setPower(gamepad2.left_stick_y * 0.5);

        }
    }

    // This is the class to control the base of the robot to move arround, this normally is
    // controlled by one person
    private class baseControl implements Runnable {
        @Override
        public void run() {
            while (!Thread.interrupted() && opModeIsActive()) {
                // Motor control logic for motors 1 and 2
                //Call Robot base movement algorithem to drive the base
                driveControl.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

                if(gamepad1.left_bumper){
                    robotLift.setPower(1);

                    // after launched wait 1.5 seconds move back to ready position
//                    sleep(1000);
                    // 0.4 is the ready position
                    //planeLaunch.setPosition(0.4);
                    sleep(1000);
                    robotLift.setPower(0);


                }
                if(gamepad1.right_bumper){
                    robotLift.setPower(-1);
                    // after launched wait 1.5 seconds move back to ready position
//                    sleep(1000);
                    // 0.4 is the ready position
                    //planeLaunch.setPosition(0.4);
                    sleep(1000);
                    robotLift.setPower(0);
                }


            }
        }
    }//end of class baseControl

    // This is the class to control arms , claws
    private class armControl implements Runnable {
        @Override
        public void run() {

            //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double lspower = 0;
            boolean moveup = false;
            boolean moveup2 = false;
            boolean moveup3 = false;
            boolean lsmove = false;
            boolean lsmove2 = false;
            boolean clawopen = false;
            boolean clawup = false;
            boolean defaultscore = false;
            boolean mediumscore = false;
            boolean highscore = false;
            boolean hang = false;
            boolean hangcount = false;


            //waitForStart();
            //set closed claw and claw lift down
            clawLeft.setPosition(1);
            clawRight.setPosition(0.6);
            clawLift.setPosition(0.173);
            //while (!isStopRequested()) {
            while (!Thread.interrupted() && opModeIsActive()) {

                //lift power take from the second game pad
                lspower = gamepad2.right_stick_y;
                lsRight.setPower(lspower);
                lsLeft.setPower(-lspower);


                /* HuskyLens.Block[] blocks = huskyLens.blocks();
                   telemetry.addData("Block count", blocks.length);
                   for (int i = 0; i < blocks.length; i++) {
                       telemetry.addData("Block", blocks[i].toString());
                } */

                //Plane launcher
                //if(gamepad2.left_trigger >=0.1){
//                if (gamepad2.y) {
//                    // 1 is the after launch position
//                    planeLaunch.setPower(1);
//                    // after launched wait 1.5 seconds move back to ready position
//                    sleep(1000);
//                    // 0.4 is the ready position
//                    //planeLaunch.setPosition(0.4);
//                    //sleep(1000);
//                    planeLaunch.setPower(0);
//                }

                //Claw contols  -  close and open, when the claw is closed, then open it, when claw is open, then close it
                if (gamepad2.left_bumper) {
                    // 1 is the after launch position
                    planeLaunch.setPower(1);
                    // after launched wait 1.5 seconds move back to ready position
                    sleep(1000);
                    // 0.4 is the ready position
                    //planeLaunch.setPosition(0.4);
                    //sleep(1000);
                    planeLaunch.setPower(0);
                }
                if (gamepad2.right_bumper) {
                    //if claw is closed then open it
                    if (clawopen == false) {
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(1);
                        sleep(250);
                    }
                    //if claw is opened then close it
                    else {
                        clawLeft.setPosition(1);
                        clawRight.setPosition(0.5);
                        sleep(250);
                    }
                    clawopen = !clawopen;
                }

                //Claw - move up and down, when its already up, move it down, when its already down, then move up
                if (gamepad2.right_trigger >= 0.1) {
                    if (clawup) {
                        clawLift.setPosition(0.173);
                        sleep(250);
                    } else {
                        clawLift.setPosition(0.525);
                        sleep(250);
                    }
                    clawup = !clawup;
                }
                //make the arm lift so we can manually reset it
                if (gamepad2.a) {
                    armLift.setPosition(0.5);
                }
                //make the arm go back down to default position on the ground
                if (gamepad2.x) {
                    armLift.setPosition(0.125);
                }
//                if(gamepad2.dpad_left){
//                    robotLift.setPower(-1);
//                    sleep(5000);
//                    robotLift.setPower(0);
//                    sleep(250);
//
//
////                    hang=!hang;
//
//                }
//                if(gamepad2.left_bumper){
//                    robotLift.setPower(1);
//                    sleep(5000);
//                    robotLift.setPower(0);
//                    sleep(250);
//                }
                if (gamepad2.dpad_up) {
                    if (defaultscore == false) {
                        //move up linear slides
                        lsRight.setPower(-1);
                        lsLeft.setPower(1);
                        sleep(250);
                        lsRight.setPower(0);
                        lsLeft.setPower(0);
                        //end move up
                        armLift.setPosition(0.8);
                        sleep(500);
                        clawLift.setPosition(1);
                        moveup = true;
                        sleep(250);
                    }
                    //automation to reset position
                    else if (defaultscore == true) {
                        if (moveup) {
                            //reset linear slides only if it was up
                            lsRight.setPower(1);
                            lsLeft.setPower(-1);
                            sleep(250);
                            lsRight.setPower(0);
                            lsLeft.setPower(0);
                            moveup = false;
                        }
                        armLift.setPosition(0.5);
                        sleep(1000);
                        clawLift.setPosition(0.8);
                        clawLeft.setPosition(1);
                        clawRight.setPosition(0.6);
                        armLift.setPosition(0.125);
                        sleep(250);
                        clawLift.setPosition(0.173);
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(1);
                        clawup = false;
                        clawopen = true;
                        sleep(250);
                    }
                    defaultscore = !defaultscore;
                }
                if (gamepad2.dpad_right) {
                    if (mediumscore == false) {
                        lsRight.setPower(-1);
                        lsLeft.setPower(1);
                        sleep(250);
                        lsRight.setPower(0);
                        lsLeft.setPower(0);
                        //end move up
                        armLift.setPosition(0.8);
                        sleep(1500);
                        clawLift.setPosition(1);
                        lsRight.setPower(-1);
                        lsLeft.setPower(1);
                        sleep(1250);
                        lsRight.setPower(0);
                        lsLeft.setPower(0);
                        moveup3 = true;
                        lsmove2 = true;
                        sleep(250);
                    } else if (mediumscore == true) {
                        armLift.setPosition(0.5);
                        sleep(1500);
                        clawLift.setPosition(0.8);
                        if (moveup3) {
                            //reset linear slides only if it was up
                            lsRight.setPower(1);
                            lsLeft.setPower(-1);
                            sleep(250);
                            lsRight.setPower(0);
                            lsLeft.setPower(0);
                            moveup3 = false;
                        }
                        if (lsmove2) {
                            lsRight.setPower(1);
                            lsLeft.setPower(-1);
                            sleep(1250);
                            lsRight.setPower(0);
                            lsLeft.setPower(0);
                            lsmove2 = false;

                        }
                        clawLeft.setPosition(1);
                        clawRight.setPosition(0.6);
                        armLift.setPosition(0.125);
                        sleep(250);
                        clawLift.setPosition(0.173);
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(1);
                        clawup = false;
                        clawopen = true;
                        sleep(250);
                    }
                    mediumscore = !mediumscore;
                }
                //automation to score pixel
                if (gamepad2.dpad_down) {
                    if (highscore == false) {
                        lsRight.setPower(-1);
                        lsLeft.setPower(1);
                        sleep(250);
                        lsRight.setPower(0);
                        lsLeft.setPower(0);
                        //end move up
                        armLift.setPosition(0.8);
                        sleep(1500);
                        clawLift.setPosition(1);
                        //linear slide go up
                        lsRight.setPower(-1);
                        lsLeft.setPower(1);
                        sleep(1500);
                        lsRight.setPower(0);
                        lsLeft.setPower(0);
                        moveup2 = true;
                        lsmove = true;
                        sleep(250);
                    } else if (highscore == true) {
                        armLift.setPosition(0.5);
                        sleep(500);
                        clawLift.setPosition(0.8);
                        if (moveup2) {
                            //reset linear slides only if it was up
                            lsRight.setPower(1);
                            lsLeft.setPower(-1);
                            sleep(250);
                            lsRight.setPower(0);
                            lsLeft.setPower(0);
                            moveup2 = false;
                        }
                        if (lsmove) {
                            lsRight.setPower(1);
                            lsLeft.setPower(-1);
                            sleep(1500);
                            lsRight.setPower(0);
                            lsLeft.setPower(0);
                            lsmove = false;
                        }
                        clawLeft.setPosition(1);
                        clawRight.setPosition(0.6);
                        armLift.setPosition(0.125);
                        sleep(250);
                        clawLift.setPosition(0.173);
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(1);
                        clawup = false;
                        clawopen = true;
                        sleep(250);
                    }
                    highscore = !highscore;
                }
                telemetry.update();
            }
        } //end of class armControl.run()
    }//end of class armControl

}//end of main class to3controlchange_ms

