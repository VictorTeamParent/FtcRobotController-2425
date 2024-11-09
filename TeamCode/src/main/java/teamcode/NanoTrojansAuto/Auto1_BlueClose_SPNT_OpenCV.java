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
import teamcode.controls_NanoTrojans1;
import teamcode.drive.SampleMecanumDrive;
import teamcode.resources_NanoTrojans1;
import teamcode.trajectorysequence.TrajectorySequence;

/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "Auto1_BlueClose_SPNT_OpenCV")
public class Auto1_BlueClose_SPNT_OpenCV extends LinearOpMode {

    // Constants for encoder counts and wheel measurements

    OpenCvWebcam webcam2;
    LCamConeLocDetection pipeline2;
    LCamConeLocDetection.LSideConePosition position2 = LCamConeLocDetection.LSideConePosition.OTHER;

    private controls_NanoTrojans1 g2control;
    private resources_NanoTrojans1 resources;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors

        // Set motor directions (adjust as needed based on your robot configuration)


        // Set motor modes

        resources = new resources_NanoTrojans1(hardwareMap);
        /*
         *  Initialize camera and set pipeline
         */
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId2);
        pipeline2 = new LCamConeLocDetection();
        webcam2.setPipeline(pipeline2);
        g2control=new controls_NanoTrojans1( resources.lsRight, resources.lsLeft, resources.planeLaunch,
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
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeLeft(36)
                        .forward(7)
                        .build();
                drive.followTrajectorySequence(trajSeq3);

                stop = true;


            } else if (position2 == LCamConeLocDetection.LSideConePosition.CENTER) {
                telemetry.addLine("Detected Cone at Center");
                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineTo(new Vector2d(36, 10), Math.toRadians(89))
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();

                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeLeft(10)
                        .forward(25)
                        //.forward(4)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                //********Parking
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence
                TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeLeft(27)
                        .forward(7)
                        .build();
                drive.followTrajectorySequence(trajSeq4);

                stop = true;

            } else if (position2 == LCamConeLocDetection.LSideConePosition.LEFT) {
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
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                //parking
                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeLeft(18)
                        .forward(7)
                        .build();
                drive.followTrajectorySequence(trajSeq3);

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







}
