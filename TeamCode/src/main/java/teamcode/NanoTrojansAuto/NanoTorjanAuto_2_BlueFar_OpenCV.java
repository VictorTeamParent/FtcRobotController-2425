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
import teamcode.trajectorysequence.TrajectorySequence;

/**
 * This class contains the Autonomous Mode program.
 */
@Config
@Autonomous(name = "Auto_2_BlueFar_OpenCV")
public class NanoTorjanAuto_2_BlueFar_OpenCV extends LinearOpMode {

    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch
    OpenCvWebcam webcam;
    RCamConeLocDetection pipeline;
    RCamConeLocDetection.RSideConePosition position = RCamConeLocDetection.RSideConePosition.OTHER;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private Servo clawLift = null;
    private Servo armLift = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private DcMotor lsRight = null;
    private DcMotor lsLeft = null;
    //private DcMotor intake = null;
    private CRServo planeLaunch = null;
    private CRServo robotLift = null;
    private DcMotor dcArm;
    private int frontLeftMotorCounts = 0;


    //The following are for single camera
    private int frontRightMotorCounts = 0;
    private int rearLeftMotorCounts = 0;
    private int rearRightMotorCounts = 0;
    private controls_NanoTrojans g2control;


    public static double parkingLongStrafe = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        rearRightMotor = hardwareMap.get(DcMotor.class, "backRight");
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

        // Set motor directions (adjust as needed based on your robot configuration)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         *  Initialize camera and set pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RCamConeLocDetection();
        webcam.setPipeline(pipeline);
        g2control=new controls_NanoTrojans(lsRight, lsLeft, planeLaunch,
                clawLeft, clawRight, clawLift, armLift, robotLift);

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
            //sleep(1000);

            position = pipeline.getPosition();
            telemetry.addData("Blue far Got position", position);
            telemetry.update();

            if (position == RCamConeLocDetection.RSideConePosition.LEFT) {

                telemetry.addLine("Detected Cone at Left");
                telemetry.update();

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(24)
                        .turn(Math.toRadians(89))
                        .back(9)
                        .forward(6)
                        .strafeLeft(8)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();

                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeLeft(18)
                        .turn(Math.toRadians(89))
                        .turn(Math.toRadians(89))
                        .forward(87)
                        .strafeLeft(25)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                doRestStuff();

                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(27)
                        .build();
                drive.followTrajectorySequence(trajSeq3);

                stop = true;


            } else if (position == RCamConeLocDetection.RSideConePosition.CENTER) {
                telemetry.addLine("Detected Cone at Center");
                telemetry.update();
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(48)
//                        .turn(Math.toRadians(90))
//                        .turn(Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(trajSeq);
                //sleep(500);
                dropTheConePixel();

                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(2)
                        .turn(-Math.toRadians(89))
                        .forward(89)
                        .strafeLeft(23)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
//                turnLeft90D5MoreD(0.8);
                doRestStuff();
                //********Parking
                TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(23)
                        .build();
                drive.followTrajectorySequence(trajSeq4);

                stop = true;

            } else if (position == RCamConeLocDetection.RSideConePosition.RIGHT) {
                telemetry.addLine("Detected Cone at Right");
                telemetry.update();

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(27)
                        .turn(-Math.toRadians(89))
                        .back(3)
                        .forward(6)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                dropTheConePixel();
                TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(25)
                        .forward(86)
                        .strafeLeft(19)
                        .build();
                drive.followTrajectorySequence(trajSeq2);
                sleep(500);
                doRestStuff();
                TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(new Pose2d())
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

        sleep(250);
        g2control.smallls();
        sleep(250);
        g2control.smalllsstop();

        g2control.armFull();
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

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }


    private void moveDistance(double inches, double power) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetPosition);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + targetPosition);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + targetPosition);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + targetPosition);


        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //double power = 0.3; // Adjust power as needed
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() &&
                frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() &&
                rearRightMotor.isBusy()) {
            // Wait for motors to reach target position
        }

        resetEncoderCounts();
        resetRobotPosition();
        stopRobot();
    }


    private void turnLeft90D(double power) {
        int turnCounts = calculateTurnCountsLeft();

        // Set target positions for motors to perform a 90-degree right turn
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + turnCounts);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - turnCounts);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + turnCounts);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() - turnCounts);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //double power = 0.8; // Adjust power as needed for turning
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() &&
                frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() &&
                rearRightMotor.isBusy()) {
            // Wait for motors to reach target position

            //telemetry.addData(" Parallel Right Encoder Current Position",parallel2.getCurrentPosition());
        }

        resetEncoderCounts();
        resetRobotPosition();
        stopRobot();
    }


    private void turnLeft90D5MoreD(double power) {
        int turnCounts = calculateTurnCountsLeft5MoreD();

        // Set target positions for motors to perform a 90-degree right turn
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + turnCounts);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - turnCounts);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + turnCounts);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() - turnCounts);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //double power = 0.8; // Adjust power as needed for turning
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() &&
                frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() &&
                rearRightMotor.isBusy()) {
            // Wait for motors to reach target position

            //telemetry.addData(" Parallel Right Encoder Current Position",parallel2.getCurrentPosition());
        }

        resetEncoderCounts();
        resetRobotPosition();
        stopRobot();
    }


    private void turnRight90D(double power) {
        int turnCounts = calculateTurnCountsRight();

        // Set target positions for motors to perform a 90-degree right turn
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - turnCounts);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + turnCounts);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() - turnCounts);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + turnCounts);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //double power = 0.5; // Adjust power as needed for turning
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() &&
                frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() &&
                rearRightMotor.isBusy()) {
            // Wait for motors to reach target position
        }

        stopRobot();
        resetEncoderCounts();
        resetRobotPosition();
    }

    private void turnRight90D5moreD(double power) {
        int turnCounts = calculateTurnCountsRight5moreD();

        // Set target positions for motors to perform a 90-degree right turn
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - turnCounts);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + turnCounts);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() - turnCounts);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + turnCounts);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //double power = 0.5; // Adjust power as needed for turning
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() &&
                frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() &&
                rearRightMotor.isBusy()) {
            // Wait for motors to reach target position
        }

        stopRobot();
        resetEncoderCounts();
        resetRobotPosition();
    }

    private void strafeRight(double inches, double power) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - targetPosition);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - targetPosition);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + targetPosition);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + targetPosition);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() && rearRightMotor.isBusy()) {
            // Wait until motors reach target position
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeLeft(double inches, double power) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetPosition);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + targetPosition);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() - targetPosition);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() - targetPosition);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() && rearRightMotor.isBusy()) {
            // Wait until motors reach target position
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int calculateTurnCountsLeft() {
        // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
        // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
        double robotWidth = 28; // This value represents half the distance between the wheels
        double wheelCircumference = Math.PI * robotWidth;
        double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
        return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
    }

    private int calculateTurnCountsLeft5MoreD() {
        // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
        // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
        double robotWidth = 29.5; // This value represents half the distance between the wheels
        double wheelCircumference = Math.PI * robotWidth;
        double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
        return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
    }

    private int calculateTurnCountsRight() {
        // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
        // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
        double robotWidth = 28.5; // This value represents half the distance between the wheels
        double wheelCircumference = Math.PI * robotWidth;
        double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
        return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
    }

    private int calculateTurnCountsRight5moreD() {
        // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
        // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
        double robotWidth = 32; // This value represents half the distance between the wheels
        double wheelCircumference = Math.PI * robotWidth;
        double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
        return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
    }

    private void stopRobot() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetEncoderCounts() {
        // Reset the encoder counts for all four motors to zero
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetRobotPosition() {
        // Reset any other variables or mechanisms used for position tracking or orientation
        frontLeftMotorCounts = 0;
        frontRightMotorCounts = 0;

        rearLeftMotorCounts = 0;
        rearRightMotorCounts = 0;
        // For encoder-based position tracking, resetting the counts is sufficient in this example
    }
}
