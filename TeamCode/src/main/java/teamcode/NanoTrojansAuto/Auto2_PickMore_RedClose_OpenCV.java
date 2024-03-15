/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.NanoTrojansAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.RCamConeLocDetection;
import teamcode.controls_NanoTrojans;
import teamcode.drive.SampleMecanumDrive;
import teamcode.resources_NanoTrojans;
import teamcode.trajectorysequence.TrajectorySequence;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;


/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "Auto2_PickMore_RedClose_OpenCV")
public class Auto2_PickMore_RedClose_OpenCV extends LinearOpMode {

    // Constants for encoder counts and wheel measurements

    OpenCvWebcam webcam;
    RCamConeLocDetection pipeline;
    RCamConeLocDetection.RSideConePosition position = RCamConeLocDetection.RSideConePosition.OTHER;

    private controls_NanoTrojans g2control;

    private resources_NanoTrojans resources;
    
    private int autoOptions = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        resources = new resources_NanoTrojans(hardwareMap);

        /*
         *  Initialize camera and set pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RCamConeLocDetection();
        webcam.setPipeline(pipeline);
        g2control=new controls_NanoTrojans( resources.lsRight, resources.lsLeft, resources.planeLaunch,
                resources.clawLeft, resources.clawRight, resources.clawLift, resources.armLift);

        /*
         *  Create a thread for camera, so it will watch for us
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        /*
         *  create an instacne for MecanumDrive car
         */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean stop = false;



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //UserChoice();

        while (opModeIsActive() && !stop) {

            g2control.closeClaw();
            g2control.clawUp();

            // Don't burn CPU cycles busy-looping in this sample
            //sleep(1000);

            position = pipeline.getPosition();
            telemetry.addData("Red Close Got position", position);
            telemetry.update();

            if (position == RCamConeLocDetection.RSideConePosition.LEFT) {
                telemetry.addLine("Detected Cone at Left");
                telemetry.update();
                TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineTo(new Vector2d(25, -5), -Math.toRadians(89))
                        .back(12)
                        .forward(5)
                        .strafeLeft(5)
                        .build();
                drive.followTrajectorySequence(traj);
                dropTheConePixel();

                Pose2d startingPose2 = traj.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .forward(36)
                        .strafeLeft(1)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                //sleep(500);
                doRestStuff();

                //parking
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(31)
                        .forward(8)
                        .build();
                drive.followTrajectorySequence(trajSeq3);
                stop = true;


            } else if (position == RCamConeLocDetection.RSideConePosition.CENTER) {
                telemetry.addLine("Detected Cone at Center");
                telemetry.update();
                Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(38, -10), -Math.toRadians(89))
                        .build();
                drive.followTrajectory(traj);
                dropTheConePixel();

                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose2 = traj.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .forward(24)
                        .strafeRight(14)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                //********Parking
                TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(25)
                        .forward(7)
                        .build();
                drive.followTrajectorySequence(trajSeq4);

                stop = true;

            } else if (position == RCamConeLocDetection.RSideConePosition.RIGHT) {
//                telemetry.addLine("Detected Cone at Right");
//                telemetry.update();
                int lluptime = 160;

                Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(32, -20), -Math.toRadians(89))
                        .build();
                drive.followTrajectory(traj);
                dropTheConePixel();

                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose2 = traj.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeRight(15.5)
                        .forward(14)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                autoOptions = 2;
                // just parking
                if (autoOptions == 1)
                {
                    //parking
                    // Update the starting pose for the second trajectory sequence
                    Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                    TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(17)
                        .forward(7)
                        .build();
                drive.followTrajectorySequence(trajSeq3);
                //pick up more pixels
              } else if(autoOptions == 2) {

                    //pick more pixels
                    Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                    TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                            .strafeLeft(36.5)
                            .back(103.8)
                            .build();
                    drive.followTrajectorySequence(trajSeq3);
                    setupLeftClawToPickStack(lluptime);

                    Pose2d startingPose4 = trajSeq3.end(); //
                    TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startingPose4)
                            .back(1.60)
                            .build();
                    drive.followTrajectorySequence(trajSeq4);

                    //sleep(1000);
                    g2control.closeLeftClaw();
                    sleep(1000);

                    g2control.openLeftClaw();

                   Pose2d startingPose5 = trajSeq4.end(); //
                    TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(startingPose5)
                            .back(0.3)
                            .build();
                    drive.followTrajectorySequence(trajSeq5);
                    sleep(100);
                    g2control.closeLeftClaw();
                    sleep(1000);


//
//                    if(detectPixel(resources.rightClawColorSensor))
//                    {
//                        //g2control.closeRightClaw();
//                        g2control.closeLeftClaw();
//                    }
//
//                    if (detectPixel(resources.leftClawColorSensor))
//                    {
//                        //g2control.closeRightClaw();
//                        g2control.openLeftClaw();
//                        Pose2d startingPose5 = trajSeq4.end(); //
//                        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(startingPose5)
//                                .back(0.3)
//                                .build();
//                        drive.followTrajectorySequence(trajSeq5);
//
//                        g2control.closeLeftClaw();
//                        sleep
//
//                    }

                    g2control.clawUp();
                    sleep(1000);

                    Pose2d startingPose6 = trajSeq5.end(); //
                    TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(startingPose6)
                            .forward(105)
                            .strafeRight(25)
                            .build();
                    drive.followTrajectorySequence(trajSeq6);
                    doRestStuff();

                    lldown(lluptime);
                }


                stop = true;
            }
        }
    }

    private void dropTheConePixel() {
        g2control.clawDown();
        sleep(500);
        g2control.openLeftClaw();
        //g2control.openClaw();
        sleep(500);
        g2control.clawUp();
        //g2control.closeClaw();
        g2control.closeLeftClaw();
    }

    private void doRestStuff() {
        //************************
        // Lift claw and setup position
        //end move up

        g2control.armFull();
        sleep(250);
        g2control.clawUp();
        //g2control.clawparallel();
        sleep(1000);
        g2control.openClaw();
        sleep(1000);

        g2control.armUp();
        sleep(500);
        g2control.clawUp();
        //sleep(500);
        g2control.closeClaw();
        g2control.armDown();
        //sleep(250);
        g2control.clawUp();

    }

    private void setupRighClawToPickStack() {
        //************************
        // Lift claw and setup position
        //end move up

        g2control.smallls();
        sleep(250);
        g2control.smalllsstop();

        g2control.clawDown();
        sleep(500);
        g2control.openRightClaw();
        sleep(500);

        //g2control.closeRightClaw();
//        g2control.clawUp();
//        g2control.reversesmallls();
//        sleep(150);
//        g2control.reversesmallls();
    }

    private void setupLeftClawToPickStack(int lluptime) {
        //************************
        // Lift claw and setup position
        //end move up

        g2control.smallls();
        sleep(lluptime);
        g2control.smalllsstop();

        g2control.openLeftClawWide();

        sleep(500);
        g2control.clawDown();
        //g2control.openRightClawWide();
        sleep(500);

        //g2control.closeRightClaw();
//        g2control.clawUp();
//        g2control.reversesmallls();
//        sleep(150);
//        g2control.reversesmallls();
    }

    private void lldown(int time) {
        //************************
        // Lift claw and setup position
        //end move up

       g2control.reversesmallls();
        sleep(time);
        g2control.reversesmallls();

    }



    private void UserChoice() {
        int userChoice = getUserInput();

        // Execute the selected autonomous behavior based on user choice
        switch (userChoice) {
            case 1:
                autoOptions = 1;
                telemetry.addLine( "You Chose Parking Only");
                break;
            case 2:
                autoOptions = 2;
                telemetry.addLine( "You Chose Pick more Pixels");
                break;
            // Add more cases for additional options as needed
            default:
                // Default behavior if an invalid option is selected
                telemetry.addData("Error", "Invalid option selected");
                telemetry.update();
                break;

        }
    }
    
    // Method to get user input (replace with your implementation)
    private int getUserInput()
    {
            // Prompt the user to make a choice and wait for input
            telemetry.addData("Autonomous Options", ":");
            telemetry.addLine( "dpad_left: Parking Only");
            telemetry.addLine("dpad_right: Get More Pixels");
            telemetry.addData("Enter Choice", "Press dpad-left or right");
            telemetry.update();

            // Loop until valid input is received
            while (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                // Check gamepad input
                if (gamepad1.dpad_left) {
                    return 1; // Option 1 selected

                } else if (gamepad1.dpad_right) {
                    return 2; // Option 2 selected
                }
            }
            return 0; // Default to 0 if no valid input is received
        }

    private boolean detectPixel (ColorSensor cs)
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
    }




