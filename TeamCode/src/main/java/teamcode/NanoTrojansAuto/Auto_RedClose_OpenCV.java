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

import teamcode.resources_base_NanoTrojans;

/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "Auto_RedClose_OpenCV")
public class Auto_RedClose_OpenCV extends LinearOpMode {

    // Constants for encoder counts and wheel measurements

    OpenCvWebcam webcam;
    RCamConeLocDetection pipeline;
    RCamConeLocDetection.RSideConePosition position = RCamConeLocDetection.RSideConePosition.OTHER;
//    private DcMotor frontLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor rearLeftMotor;
//    private DcMotor rearRightMotor;
//    private Servo clawLift = null;
//    private Servo armLift = null;
//    private Servo clawLeft = null;
//    private Servo clawRight = null;
//    private DcMotor lsRight = null;
//    private DcMotor lsLeft = null;
//    //private DcMotor intake = null;
//    private CRServo planeLaunch = null;
//    private CRServo robotLift = null;
//    private int frontLeftMotorCounts = 0;
//
//
//    //The following are for single camera
//    private int frontRightMotorCounts = 0;
//    private int rearLeftMotorCounts = 0;
//    private int rearRightMotorCounts = 0;
    private controls_NanoTrojans g2control;

//    private DcMotor dcArm;

    private resources_NanoTrojans resources;
    //private resources_base_NanoTrojans resourcesbase;
    //public static double parkingLongStrafe = 30;

    @Override
    public void runOpMode() throws InterruptedException {

        //resourcesbase = new resources_base_NanoTrojans(hardwareMap);
        resources = new resources_NanoTrojans(hardwareMap);
        // Initialize motors
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
//        rearLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
//        rearRightMotor = hardwareMap.get(DcMotor.class, "backRight");
//        lsRight = hardwareMap.dcMotor.get("lsRight");
//        lsLeft = hardwareMap.dcMotor.get("lsLeft");
//        //intake = hardwareMap.dcMotor.get("intake");
//
//        //Servo Motors
//        planeLaunch = hardwareMap.crservo.get("planeLaunch");
//        robotLift = hardwareMap.crservo.get("robotLift");
//
//        //hang
//
//
//        // get 2 claw motors
//        clawLeft = hardwareMap.servo.get("clawLeft");
//        clawRight = hardwareMap.servo.get("clawRight");
//
//        // get 2 arm motors
//        clawLift = hardwareMap.servo.get("clawLift");
//        armLift = hardwareMap.servo.get("armLift");
//        dcArm = hardwareMap.dcMotor.get("dcArm");
//
//
//        // Set motor directions (adjust as needed based on your robot configuration)
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);




        // Set motor modes
        //setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        waitForStart();

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

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(26)
                        .turn(-Math.toRadians(89))
                        .back(8)
                        .forward(5)
                        .strafeLeft(2)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(37)
                        .strafeLeft(6)
                        //.forward(6)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                //sleep(500);
                doRestStuff();
                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(31)
                        .forward(6)
                        .build();
                drive.followTrajectorySequence(trajSeq3);

                stop = true;


            } else if (position == RCamConeLocDetection.RSideConePosition.CENTER) {
                telemetry.addLine("Detected Cone at Center");
                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())

                        .forward(49)
                        .build();
                drive.followTrajectorySequence(trajSeq);

                dropTheConePixel();

                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(3)
                        .turn(-Math.toRadians(89))
                        .forward(30)
                        .strafeRight(25)
                        .forward(6)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();
                //********Parking
                TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(24)
                        .forward(7)
                        .build();
                drive.followTrajectorySequence(trajSeq4);

                stop = true;

            } else if (position == RCamConeLocDetection.RSideConePosition.RIGHT) {
                telemetry.addLine("Detected Cone at Right");
                telemetry.update();

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(28)
                        .turn(-Math.toRadians(89))
                        .forward(21)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();

                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose2 = trajSeq.end(); // Use the end pose of the first sequence as the starting pose for the second sequence


                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startingPose2)
                        .strafeRight(9)
                        .forward(15)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                // Update the starting pose for the second trajectory sequence
                Pose2d startingPose3 = trajSeq2.end(); // Use the end pose of the first sequence as the starting pose for the second sequence

                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startingPose3)
                        .strafeRight(19)
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
        //sleep(500);
        //g2control.openClaw();


    }

}
