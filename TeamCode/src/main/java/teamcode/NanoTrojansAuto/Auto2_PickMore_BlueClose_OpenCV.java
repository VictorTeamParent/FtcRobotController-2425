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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.LCamConeLocDetection;
import teamcode.controls_NanoTrojans;
import teamcode.drive.SampleMecanumDrive;
import teamcode.resources_NanoTrojans;
import teamcode.trajectorysequence.TrajectorySequence;

/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "Auto2_PickMore_BlueClose_OpenCV")
public class Auto2_PickMore_BlueClose_OpenCV extends LinearOpMode {

    // Constants for encoder counts and wheel measurements

    OpenCvWebcam webcam2;
    LCamConeLocDetection pipeline2;
    LCamConeLocDetection.LSideConePosition position2 = LCamConeLocDetection.LSideConePosition.OTHER;

    private controls_NanoTrojans g2control;
    private resources_NanoTrojans resources;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors

        // Set motor directions (adjust as needed based on your robot configuration)


        // Set motor modes

        resources = new resources_NanoTrojans(hardwareMap);
        /*
         *  Initialize camera and set pipeline
         */
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId2);
        pipeline2 = new LCamConeLocDetection();
        webcam2.setPipeline(pipeline2);
        g2control=new controls_NanoTrojans( resources.lsRight, resources.lsLeft, resources.planeLaunch,
                resources.clawLeft, resources.clawRight, resources.clawLift, resources.armLift);

        /*
         *  Create a thread for camera, so it will watch for us
         */
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        /*
         *  create an instacne for MecanumDrive car
         */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        boolean stop = false;

        waitForStart();

        while (opModeIsActive() && !stop) {

            g2control.closeClaw();
            g2control.clawUp();

            // Don't burn CPU cycles busy-looping in this sample
            //sleep(1000);

            position2 = pipeline2.getPosition();
            telemetry.addData("Blue Close Got position", position2);
            telemetry.update();

            if (position2 == LCamConeLocDetection.LSideConePosition.RIGHT) {
                int lluptime = 170;
                telemetry.addLine("Detected Cone at Right");
                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineTo(new Vector2d(25, 9), Math.toRadians(89))
                        .back(16)
                        .forward(4)
                        .strafeRight(2)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();

                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .forward(38.5)
                        .strafeRight(8)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                sleep(500);
                doRestStuff();

                //parking
                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(14.5)
                        .back(104.9)
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
                sleep(500);

                g2control.openLeftClaw();

                Pose2d startingPose5 = trajSeq4.end(); //
                TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(startingPose5)
                        .back(0.55)
                        .build();
                drive.followTrajectorySequence(trajSeq5);
                sleep(100);
                g2control.closeLeftClaw();
                sleep(1000);


                g2control.clawUp();
                sleep(500);

                Pose2d startingPose6 = trajSeq5.end(); //
                TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(startingPose6)
                        .forward(105.5)
                        .strafeLeft(26.5)
                        .build();
                drive.followTrajectorySequence(trajSeq6);
                doRestStuff();

                lldown(lluptime);

                stop = true;


            } else if (position2 == LCamConeLocDetection.LSideConePosition.CENTER) {
                int lluptime = 170;
                telemetry.addLine("Detected Cone at Center");


                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineTo(new Vector2d(35, 10), Math.toRadians(89))
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();

                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeLeft(10)
                        .forward(23)
                        //.forward(4)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                //more pixels

                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(23.5)
                        .back(104.5)
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
                sleep(500);

                g2control.openLeftClaw();

                Pose2d startingPose5 = trajSeq4.end(); //
                TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(startingPose5)
                        .back(0.3)
                        .build();
                drive.followTrajectorySequence(trajSeq5);
                sleep(100);
                g2control.closeLeftClaw();
                sleep(1000);


                g2control.clawUp();
                sleep(500);

                Pose2d startingPose6 = trajSeq5.end(); //
                TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(startingPose6)
                        .forward(106)
                        .strafeLeft(21)
                        .build();
                drive.followTrajectorySequence(trajSeq6);
                doRestStuff();

                lldown(lluptime);

                stop = true;

            } else if (position2 == LCamConeLocDetection.LSideConePosition.LEFT) {
                int lluptime = 170;
                telemetry.addLine("Detected Cone at LEFT");
                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineTo(new Vector2d(30, 20.5), Math.toRadians(89))
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();

                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeLeft(11)
                        .forward(14)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                //more pixels
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(31.5)
                        .back(104.9)
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
                sleep(500);

                g2control.openLeftClaw();

                Pose2d startingPose5 = trajSeq4.end(); //
                TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(startingPose5)
                        .back(0.55)
                        .build();
                drive.followTrajectorySequence(trajSeq5);
                sleep(100);
                g2control.closeLeftClaw();
                sleep(1000);


                g2control.clawUp();
                sleep(500);

                Pose2d startingPose6 = trajSeq5.end(); //
                TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(startingPose6)
                        .forward(106.5)
                        .strafeLeft(29.5)
                        .build();
                drive.followTrajectorySequence(trajSeq6);
                doRestStuff();

                lldown(lluptime);

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
        //sleep(500);
        //g2control.openClaw();

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

    }

    private void lldown(int time) {
        //************************
        // Lift claw and setup position
        //end move up

        g2control.reversesmallls();
        sleep(time);
        g2control.reversesmallls();

    }







}
