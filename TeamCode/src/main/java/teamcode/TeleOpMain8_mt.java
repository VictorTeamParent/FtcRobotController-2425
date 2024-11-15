//package org.firstinspires.ftc.teamcode;
package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "TeleOpMain8_mt", group = "TeleOp")


public class TeleOpMain8_mt extends LinearOpMode {

      private final double driveAdjuster = 1;

    // the following are for huskylen

    private final int READ_PERIOD = 1;


    Orientation angles;

    private DriveControl_NanoTorjan driveControl;
    //private DriveControl driveControl;

    private controls_NanoTrojans1 g2control;

    private resources_NanoTrojans1 resources;

    private boolean rightPixelPicked = false;
    private boolean leftPixelPicked = false;
    private boolean autopick = false;


    @Override
    public void runOpMode()  throws InterruptedException {
        resources=new resources_NanoTrojans1(hardwareMap);
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        g2control=new controls_NanoTrojans1(resources.lsRight, resources.lsLeft, resources.planeLaunch,
                resources.clawLeft, resources.clawRight, resources.clawLift, resources.armLift);


        //Thread baseControlThread = new Thread(new baseControl());
        Thread armControlThread = new Thread(new armControl());
        Thread lsControlThread = new Thread(new lsControl());
        Thread lsControl2 = new Thread(new lsControl2());

        //Start 2  threads
        //baseControlThread.start();
        armControlThread.start();
        lsControl2.start();
        //lsControlThread.start();

        // This is the 3rd thread
        //The following  loop is just to keep this main thread running.
        // Add above 2 threads basicall we have 3 threads running
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            if ( gamepad1.y ){
                telemetry.addLine("Y pressed");
                g2control.planeLaunch();
                sleep(100);
                g2control.planeLaunchstop();
                //droneLaunced = true;
            }
        }

    }

    // The following is developed by NT , but we are not using it now
    // This is the thread class to control the base of the robot to move arround, this normally is
    // controlled by another person seperated from the base control person
    private class baseControl implements Runnable {
        boolean droneLaunced = false;
        @Override
        public void run() {
            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                // Motor control logic for motors 1 and 2
                //Call Robot base movement algorithem to drive the base
                driveControl.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

                if ( gamepad1.y ){
                    g2control.planeLaunch();
                    sleep(100);
                    g2control.planeLaunchstop();
                    //droneLaunced = true;
                }



            }
        }
    }//end of class baseControl

    private class lsControl2 implements Runnable {
        boolean clawClosed = false;
        @Override
        public void run() {

            boolean moveup2 = false;
            boolean lsmove = false;
            boolean lowscore = false;

            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {
                if (gamepad2.dpad_left) {
                    if (lowscore == false) {
                        //move up linear slides
                        //end move up
                        g2control.armFull();
                        sleep(500);

                        g2control.clawparallel();
                        sleep(250);
                    }
                    //automation to reset position
                    else if (lowscore == true) {

                        g2control.armUp();
                        sleep(1000);
                        g2control.clawUp();
                        g2control.closeClaw();
                        g2control.armDown();
                        sleep(250);
                        g2control.clawUp();
                        g2control.openClaw();
                        sleep(250);
                    }
                    lowscore = !lowscore;
                }



            }
        }
    }//end of class baseControl

    private class lsControl implements Runnable {
        boolean clawClosed = false;
        @Override
        public void run() {

            boolean moveup2 = false;
            boolean lsmove = false;
            boolean highscore = false;

            waitForStart();
            while (!Thread.interrupted() && opModeIsActive()) {

                double lspower = gamepad2.right_stick_y;
                resources.lsRight.setPower(lspower);
                resources.lsLeft.setPower(-lspower);

            }
        }
    }//end of class baseControl


    // This is the thread class to control arms , claws
    private class armControl implements Runnable {

        boolean rightpixeldetected;
        boolean leftpixeldetected;



        @Override
        public void run() {

            boolean moveup2 = false;
            boolean moveup3 = false;
            boolean lsmove = false;
            boolean lsmove2 = false;
            boolean clawopen = false;
            boolean clawup = true;
            boolean defaultscore = false;
            boolean mediumscore = false;
            boolean highscore = false;
            boolean leftclawopen = false;
            boolean rightclawopen = false;
            boolean lowscore = false;
            double lspower = 0;
            //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

             waitForStart();
            //set closed claw and claw lift down
            //while (!isStopRequested()) {
            while (!Thread.interrupted() && opModeIsActive()) {
                double clawliftpos = resources.clawLift.getPosition();
                double armliftpos = resources.armLift.getPosition();

                //lift power take from the second game pad
                lspower = gamepad2.right_stick_y;
                resources.lsRight.setPower(lspower);
                resources.lsLeft.setPower(-lspower);

                //Claw contols  -  close and open, when the claw is closed, then open it, when claw is open, then close it
                if (gamepad2.right_bumper) {
                    if (leftclawopen) {
                        g2control.closeLeftClaw();
                        sleep(250);
                    }
                    else {
                        g2control.openLeftClaw();
                        sleep(250);
                    }
                    leftclawopen=!leftclawopen;

                }

                if (gamepad2.left_bumper){
                    if(rightclawopen){
                        g2control.closeRightClaw();
                        sleep(250);
                    }
                    else{
                        g2control.openRightClaw();
                        sleep(250);
                    }
                    rightclawopen= !rightclawopen;
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
                //make the arm lift so we can manually reset it
                if (gamepad2.a) {
                    g2control.armFull();

                }
                //make the arm go back down to default position on the ground
                if (gamepad2.x) {
                    g2control.armDown();

                }
                if (gamepad2.dpad_up){
                    g2control.smallls();
                    sleep(170);
                    g2control.smalllsstop();
                }
                if (gamepad2.dpad_right){
                    g2control.clawparallel();
                }
//                if (gamepad2.dpad_left) {
//                    if (lowscore == false) {
//                        //move up linear slides
//                        //end move up
//                        g2control.armFull();
//                        sleep(500);
//
//                        g2control.clawparallel();
//                        sleep(250);
//                    }
//                    //automation to reset position
//                    else if (lowscore == true) {
//
//                        g2control.armUp();
//                        sleep(1000);
//                        g2control.clawUp();
//                        g2control.closeClaw();
//                        g2control.armDown();
//                        sleep(250);
//                        g2control.clawDown();
//                        g2control.openClaw();
//                        clawup = false;
//                        clawopen = true;
//                        rightclawopen= true;
//                        leftclawopen = true;
//                        sleep(250);
//                    }
//                    lowscore = !lowscore;
//                }
//                if (gamepad2.dpad_up) {
//                    if (defaultscore == false) {
//                        //move up linear slides
//                        g2control.armFull();
//                        sleep(500);
//                        g2control.clawparallel();
//                        sleep(250);
//                        g2control.smallls();
//
//                        sleep(250);
//                        g2control.smalllsstop();
//
//                        //end move up
//
//                        lsmove=true;
//                    }
//                    //automation to reset position
//                    else if (defaultscore == true) {
//
//                        //Victor comment out this code and move to run it later
////                        if (lsmove){
////                            g2control.reversesmallls();
////                            sleep(250);
////                            g2control.reversesmalllsstop();
////                        }
//
//                        g2control.armUp();
//                        sleep(1000);
//
//                        //victor move above code to here
//                        if (lsmove){
//                            g2control.reversesmallls();
//                            sleep(250);
//                            g2control.reversesmalllsstop();
//                        }
//                        g2control.clawUp();
//                        g2control.closeClaw();
//                        g2control.armDown();
//                        sleep(250);
//                        g2control.clawDown();
//                        g2control.openClaw();
//                        clawup = false;
//                        clawopen = true;
//                        rightclawopen= true;
//                        leftclawopen = true;
//                        lsmove=false;
//                        sleep(250);
//                    }
//                    defaultscore = !defaultscore;
//                }
//                if (gamepad2.dpad_right) {
//                    if (mediumscore == false) {
//                        g2control.armFull();
//                        sleep(1500);
//                        g2control.clawparallel();
//                        g2control.smallls();
//                        sleep(250);
//                        g2control.smalllsstop();
//                        //end move up
//                        g2control.mediumls();
//                        sleep(750);
//                        g2control.mediumlsstop();
//                        moveup3 = true;
//                        lsmove2 = true;
//                        sleep(250);
//                    } else if (mediumscore == true) {
//                        g2control.armUp();
//                        sleep(1500);
//                        g2control.clawUp();
//                        if (moveup3) {
//                            //reset linear slides only if it was up
//                            g2control.reversesmallls();
//                            sleep(250);
//                            g2control.reversesmalllsstop();
//                            moveup3 = false;
//                        }
//                        if (lsmove2) {
//                            g2control.reversemediumls();
//                            sleep(750);
//                            g2control.reversemediumlsstop();
//                            lsmove2 = false;
//
//                        }
//                        g2control.closeClaw();
//                        g2control.armDown();
//                        sleep(250);
//                        g2control.clawDown();
//                        g2control.openClaw();
//                        clawup = false;
//                        clawopen = true;
//                        rightclawopen= true;
//                        leftclawopen = true;
//                        sleep(250);
//                    }
//                    mediumscore = !mediumscore;
//                }
//                //automation to score pixel
//                if (gamepad2.dpad_down) {
//                    if (highscore == false) {
//                        g2control.armFull();
//                        sleep(1250);
//                        g2control.clawparallel();
//                        g2control.smallls();
//                        sleep(250);
//                        g2control.smalllsstop();
//                        //end move up
//
//                        //linear slide go up
//                        g2control.highls();
//                        sleep(1400);
//                        g2control.highlsstop();
//                        moveup2 = true;
//                        lsmove = true;
//                        sleep(250);
//                    } else if (highscore == true) {
//                        g2control.armUp();
//                        sleep(500);
//                        g2control.clawUp();
//                        if (moveup2) {
//                            //reset linear slides only if it was up
//                            g2control.reversesmallls();
//                            sleep(250);
//                            g2control.reversesmalllsstop();
//                            moveup2 = false;
//                        }
//                        if (lsmove) {
//                            g2control.reversehighls();
//                            sleep(650);
//                            g2control.reversehighlsstop();
//                            lsmove = false;
//                        }
//                        g2control.closeClaw();
//                        g2control.armDown();
//                        sleep(250);
//                        g2control.clawDown();
//                        g2control.openClaw();
//                        clawup = false;
//                        clawopen = true;
//                        rightclawopen= true;
//                        leftclawopen = true;
//                        sleep(250);
//                    }
//                    highscore = !highscore;
//                }
                boolean enableTel = false;

                if(enableTel) {
                    if (rightclawopen) {
                        telemetry.addLine("Right claw open");
                    }
                    if (rightclawopen == false) {
                        telemetry.addLine("Right claw closed");
                    }
                    if (leftclawopen) {
                        telemetry.addLine("Left claw open");
                    }
                    if (leftclawopen == false) {
                        telemetry.addLine("Left claw closed");
                    }
                    if (!g2control.clawdown) {
                        telemetry.addLine("Claw up");
                    }
                    if (g2control.clawdown) {
                        telemetry.addLine("Claw down");
                    }
                    telemetry.addData("Arm position:", armliftpos);
                    telemetry.addData("Claw position:", clawliftpos);


                    telemetry.update();

                }
               //if(!g2control.autopicksucess) {
                   rightpixeldetected = detectPixel(resources.rightClawColorSensor);
                   leftpixeldetected = detectPixel(resources.leftClawColorSensor);


//                double distanceleft = leftClawDistanceSensor.getDistance(DistanceUnit.MM);
//                //double distanceright = rightClawDistanceSensor.getDistance(DistanceUnit.MM);
//                // Display the detected distance
//                telemetry.addData("Distance left (MM)", distanceleft);
//                //telemetry.addData("Distance Right (MM)", distanceright);
//                telemetry.addData("Claw is down ? ",  g2control.clawdown);
//
//                telemetry.update();
//
//                if(distanceleft<=25 && g2control.clawdown){
//                    //telemetry.addLine("Distance less than 30");
//                    g2control.closeLeftClaw();
//                    leftPixelPicked = true;
//                }

//                if(distanceright<=26 && g2control.clawdown){
//
//                    //telemetry.addLine("Distance less than 30");
//                    g2control.closeRightClaw();
//                    rightPixelPicked = true;
//                }

                   if (leftpixeldetected && g2control.clawdown) {

                       //telemetry.addLine("Distance less than 30");
                       g2control.closeLeftClaw();
                       leftPixelPicked = true;
                       leftpixeldetected = false;
                       leftclawopen = false;
                   }
                   if (rightpixeldetected && g2control.clawdown) {

                       //telemetry.addLine("Distance less than 30");
                       g2control.closeRightClaw();
                       rightPixelPicked = true;
                       rightpixeldetected = false;
                       rightclawopen = false;
                   }


//                   if (rightPixelPicked && leftPixelPicked) {
//                       sleep(200);
//                       g2control.clawFull();
//
//                       leftPixelPicked = false;
//                       rightPixelPicked = false;
//
//                       g2control.autopicksucess = true;
//                       //autopick = false;
//                   }

               //}


            }
        } //end of class armControl.run()

        private boolean detectPixel (ColorSensor  cs)
        {
            boolean rc = false;
            boolean enableTelemetry = true;
            int red = cs.red();
            int green = cs.green();
            int blue = cs.blue();

            //boolean rightpixeldetected = false;
            if(enableTelemetry) {
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue", blue);
                telemetry.update();
            }
//            if (red > 100 && blue > 100 && green < 50) {
//                if(enableTelemetry) {
//                    telemetry.addData("Color", "Purple");
//                }
//                rc = true;
//            }
//            // Check for yellow color
//            else if (red > 100 && green > 100 && blue < 50) {
//                if(enableTelemetry) {
//                    telemetry.addData("Color", "Yellow");
//                }
//                rc = true;
//            }
            // Check for green color
            //else if (green > 250 && red < 200 && blue > 200) {
            if ( red > 150 && green > 250 && blue > 200) {
                if(enableTelemetry) {
                    telemetry.addData("Color", "Green");
                }
                rc = true;
            }
            // Check for white color
            else if (red > 200 && green > 200 && blue > 200) {
                if(enableTelemetry) {
                    telemetry.addData("Color", "White");
                }
                rc = true;
            }
            // None of the specified colors detected
            else {
                if(enableTelemetry) {
                    telemetry.addData("Color", "Unknown");
                }
            }

            telemetry.update();
            return rc;
        }
    }//end of class armControl

}//end of main class to3controlchange_ms

