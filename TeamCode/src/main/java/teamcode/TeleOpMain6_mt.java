//package org.firstinspires.ftc.teamcode;
package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOpMain6_mt", group = "TeleOp")


public class TeleOpMain6_mt extends LinearOpMode {

    //private DcMotor intake = null;
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

    private controls_NanoTrojans g2control;
    @Override
    public void runOpMode()  throws InterruptedException {
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
        g2control=new controls_NanoTrojans(lsRight, lsLeft, planeLaunch,
                                          clawLeft, clawRight, clawLift, armLift, robotLift);

        waitForStart();
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
            //intake.setPower(gamepad2.left_stick_y * 0.5);

        }
    }

    // This is the thread class to control the base of the robot to move arround, this normally is
    // controlled by another person seperated from the base control person
    private class baseControl implements Runnable {
        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // Motor control logic for motors 1 and 2
                //Call Robot base movement algorithem to drive the base
                driveControl.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

                if(gamepad1.left_bumper){
                    g2control.hangSpin();
                    sleep(1000);
                    g2control.hangSpinstop();
                }
                if(gamepad1.right_bumper){
                    g2control.reversehangSpin();
                    sleep(1000);
                    g2control.reversehangSpinstop();

                }
            }
        }
    }//end of class baseControl

    // This is the thread class to control arms , claws
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
            boolean clawopen = true;
            boolean clawup = true;
            boolean defaultscore = false;
            boolean mediumscore = false;
            boolean highscore = false;
            boolean hang = false;
            boolean hangcount = false;
            boolean leftclawopen = false;
            boolean rightclawopen = false;

            waitForStart();
            //set closed claw and claw lift down
            //while (!isStopRequested()) {
            while (!Thread.interrupted() && opModeIsActive()) {

                //lift power take from the second game pad
                lspower = gamepad2.right_stick_y;
                lsRight.setPower(lspower);
                lsLeft.setPower(-lspower);

                //Claw contols  -  close and open, when the claw is closed, then open it, when claw is open, then close it
                if (gamepad2.left_bumper) {
                    if (leftclawopen) {
                        g2control.closeLeftClaw();
                        sleep(250);
                    }
                    else {
                        g2control.openLeftClaw();
                        sleep(250);
                    }

                }
                if (gamepad2.y){
                    g2control.planeLaunch();
                    sleep(1000);
                    g2control.planeLaunchstop();
                }
                if (gamepad2.right_bumper){
                    if(rightclawopen){
                        g2control.closeRightClaw();
                        sleep(250);
                    }
                    else{
                        g2control.openRightClaw();
                        sleep(250);
                    }
                }
                if (gamepad2.left_trigger>=0.1) {
                    //if claw is closed then open it
                    if (clawopen == false) {
                        g2control.openClaw();
                        sleep(250);
                    }
                    //if claw is opened then close it
                    else {
                        g2control.closeClaw();
                        sleep(250);
                    }
                    clawopen = !clawopen;
                    rightclawopen= !rightclawopen;
                    leftclawopen = !leftclawopen;
                }

                //Claw - move up and down, when its already up, move it down, when its already down, then move up
                if (gamepad2.right_trigger >= 0.1) {
                    if (clawup) {
                        g2control.clawDown();
                        sleep(250);
                    } else {
                        g2control.clawFull();
                        sleep(250);
                    }
                    clawup = !clawup;
                }
                if(gamepad2.left_bumper){
                    g2control.planeLaunch();
                    sleep(1000);
                    g2control.planeLaunchstop();

                }
                //make the arm lift so we can manually reset it
                if (gamepad2.a) {
                    g2control.armUp();
                }
                //make the arm go back down to default position on the ground
                if (gamepad2.x) {
                    g2control.armDown();
                }
                if (gamepad2.dpad_up) {
                    if (defaultscore == false) {
                        //move up linear slides
                        g2control.smallls();
                        sleep(250);
                        g2control.smalllsstop();
                        //end move up
                        g2control.armFull();
                        sleep(500);
                        g2control.clawUp();
                        moveup = true;
                        sleep(250);
                    }
                    //automation to reset position
                    else if (defaultscore == true) {
                        if (moveup) {
                            //reset linear slides only if it was up
                            g2control.reversesmallls();
                            sleep(250);
                            g2control.reversesmalllsstop();
                            moveup = false;
                        }
                        g2control.armUp();
                        sleep(1000);
                        g2control.clawUp();
                        g2control.closeClaw();
                        g2control.armDown();
                        sleep(250);
                        g2control.clawDown();
                        g2control.openClaw();
                        clawup = false;
                        clawopen = true;
                        rightclawopen= true;
                        leftclawopen = true;
                        sleep(250);
                    }
                    defaultscore = !defaultscore;
                }
                if (gamepad2.dpad_right) {
                    if (mediumscore == false) {
                        g2control.smallls();
                        sleep(250);
                        g2control.smalllsstop();
                        //end move up
                        g2control.armFull();
                        sleep(1500);
                        g2control.clawFull();
                        g2control.mediumls();
                        sleep(1250);
                        g2control.mediumlsstop();
                        moveup3 = true;
                        lsmove2 = true;
                        sleep(250);
                    } else if (mediumscore == true) {
                        g2control.armUp();
                        sleep(1500);
                        g2control.clawUp();
                        if (moveup3) {
                            //reset linear slides only if it was up
                            g2control.reversesmallls();
                            sleep(250);
                            g2control.reversesmalllsstop();
                            moveup3 = false;
                        }
                        if (lsmove2) {
                            g2control.reversemediumls();
                            sleep(1250);
                            g2control.reversemediumlsstop();
                            lsmove2 = false;

                        }
                        g2control.closeClaw();
                        g2control.armDown();
                        sleep(250);
                        g2control.clawDown();
                        g2control.openClaw();
                        clawup = false;
                        clawopen = true;
                        rightclawopen= true;
                        leftclawopen = true;
                        sleep(250);
                    }
                    mediumscore = !mediumscore;
                }
                //automation to score pixel
                if (gamepad2.dpad_down) {
                    if (highscore == false) {
                        g2control.smallls();
                        sleep(250);
                        g2control.smalllsstop();
                        //end move up
                        g2control.armFull();
                        sleep(1500);
                        g2control.clawFull();
                        //linear slide go up
                        g2control.highls();
                        sleep(2000);
                        g2control.highlsstop();
                        moveup2 = true;
                        lsmove = true;
                        sleep(250);
                    } else if (highscore == true) {
                        g2control.armUp();
                        sleep(500);
                        g2control.clawUp();
                        if (moveup2) {
                            //reset linear slides only if it was up
                            g2control.reversesmallls();
                            sleep(250);
                            g2control.reversesmalllsstop();
                            moveup2 = false;
                        }
                        if (lsmove) {
                            g2control.reversehighls();
                            sleep(2000);
                            g2control.reversehighlsstop();
                            lsmove = false;
                        }
                        g2control.closeClaw();
                        g2control.armDown();
                        sleep(250);
                        g2control.clawDown();
                        g2control.openClaw();
                        clawup = false;
                        clawopen = true;
                        rightclawopen= true;
                        leftclawopen = true;
                        sleep(250);
                    }
                    highscore = !highscore;
                }
                telemetry.update();
            }
        } //end of class armControl.run()
    }//end of class armControl

}//end of main class to3controlchange_ms

