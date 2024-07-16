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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "AutoNT_1_BlueFar_OpenCV")
public class AutoNT_1_BlueFar_OpenCV extends LinearOpMode {

    // Constants for encoder counts and wheel measurements

    OpenCvWebcam webcam;
    RCamConeLocDetection pipeline;
    RCamConeLocDetection.RSideConePosition position = RCamConeLocDetection.RSideConePosition.OTHER;

    private controls_NanoTrojans g2control;
    private resources_NanoTrojans resources;

    public static double parkingLongStrafe = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors

        resources = new resources_NanoTrojans(hardwareMap);
        /*
         *  Initialize camera and set pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RCamConeLocDetection();
        webcam.setPipeline(pipeline);
        g2control=new controls_NanoTrojans(resources.lsRight, resources.lsLeft, resources.planeLaunch,
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
        boolean stop = false;

        waitForStart();

        while (opModeIsActive() && !stop) {

            g2control.closeClaw();
            g2control.clawUp();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(6000);

            position = pipeline.getPosition();
            telemetry.addData("Blue far Got position", position);
            telemetry.update();

            if (position == RCamConeLocDetection.RSideConePosition.LEFT) {

                telemetry.addLine("Detected Cone at Left");
                telemetry.update();

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(24)
                        .turn(-Math.toRadians(89))
                        .back(9)
                        .forward(6)
                        .strafeLeft(8)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();
                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence


                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeLeft(18)
                        .turn(Math.toRadians(89))
                        .turn(Math.toRadians(89))
                        .forward(83)
                        .strafeLeft(30)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(37)
                        .build();
                drive.followTrajectorySequence(trajSeq3);

                stop = true;


            } else if (position == RCamConeLocDetection.RSideConePosition.CENTER) {
                telemetry.addLine("Detected Cone at Center");
                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(48)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                //sleep(500);
                dropTheConePixel();
                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence


                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .forward(2)
                        .turn(Math.toRadians(89))
                        .forward(85.5)
                        .strafeLeft(26.85)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
//                turnLeft90D5MoreD(0.8);
                doRestStuff();
                //********Parking
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence


                TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(23)
                        .build();
                drive.followTrajectorySequence(trajSeq4);

                stop = true;

            } else if (position == RCamConeLocDetection.RSideConePosition.RIGHT) {
                telemetry.addLine("Detected Cone at Right");
                telemetry.update();

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(27)
                        .turn(Math.toRadians(89))
                        .back(3)
                        .forward(6)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();
                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence


                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeRight(25)
                        .forward(84)
                        .strafeLeft(20)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                sleep(500);
                doRestStuff();
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence


                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(18)
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
        sleep(1000);
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
        g2control.smallls();
        sleep(350);
        g2control.smalllsstop();


        g2control.clawUp();
        sleep(2000);
        g2control.openClaw();
        sleep(500);







        g2control.armUp();
        sleep(500);
        g2control.clawUp();
        //sleep(500);
        g2control.closeClaw();
        g2control.armDown();
        //sleep(250);
        g2control.clawUp();
        sleep(200);
        //g2control.openClaw();

        g2control.reversesmallls();
        sleep(250);
        g2control.reversehighlsstop();

    }


}
