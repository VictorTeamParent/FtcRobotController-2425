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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.RedConeLocDetection;
import teamcode.drive.SampleMecanumDrive;
import teamcode.trajectorysequence.TrajectorySequence;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name = "Auto_Strafe_test")
public class NanoTorjanAuto_StrafeTest extends LinearOpMode {

    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch
    OpenCvWebcam webcam;
    RedConeLocDetection pipeline;
    RedConeLocDetection.RedConePosition position = RedConeLocDetection.RedConePosition.LEFT;
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
    private int frontLeftMotorCounts = 0;


    //The following are for single camera
    private int frontRightMotorCounts = 0;
    private int rearLeftMotorCounts = 0;
    private int rearRightMotorCounts = 0;
    private RedConeLocDetection RedConeLocDetector;



    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        rearRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        lsRight = hardwareMap.dcMotor.get("lsRight");
        lsLeft = hardwareMap.dcMotor.get("lsLeft");

        //Servo Motors

        // get 2 claw motors
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight= hardwareMap.servo.get("clawRight");

        // get 2 arm motors
        clawLift = hardwareMap.servo.get("clawLift");
        armLift= hardwareMap.servo.get("armLift");

        // Set motor directions (adjust as needed based on your robot configuration)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //openCvWebCamera1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // openCvWebCamera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        /**
         * Here we use a special factory method that accepts multiple WebcamName arguments. It returns an
         * {@link OpenCvSwitchableWebcam} which contains a couple extra methods over simply an {@link OpenCvCamera}.
         */


        // the following is for one camera

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RedConeLocDetection();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

//        frontLeftMotor.setPower(1);
//        frontRightMotor.setPower(1);
//        rearLeftMotor.setPower(1);
//        rearRightMotor.setPower(1);
        //RedConeLocDetector = new RedConeLocDetection();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
//                .forward(25)
//                .build();



//
//        Pose2d startPose = new Pose2d(0, 0, 0);
//        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .forward(25)
//                .turn(Math.toRadians(90))
//                .forward(30)
//                .strafeRight(Math.toRadians(12))
//
//                .build();
//        drive.followTrajectorySequence(trajSeq);

        waitForStart();
//        drive.followTrajectory(trajectory);
        boolean stop = false;
        while (opModeIsActive() && !stop) {
            telemetry.addData("Analysis", pipeline.getPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);


            //strafeRight(48, 0.8);

            strafeLeft(48, 0.8);

////            strafeRight(48, 1);
//            sleep(500);
//            strafeLeft(50, 0.2);

//            moveDistance(50, 0.8);
//
//            sleep(250);
//            moveDistance(12, 0.3);
//
//
//          // Stop the robot


                stop = true;
            }

        }


       private void moveUpLSLow()
       {
           //move up linear slides
           lsRight.setPower(-1);
           lsLeft.setPower(1);
           sleep(250);
           lsRight.setPower(0);
           lsLeft.setPower(0);
           //end move up

       }

       private void liftArm()
       {
           armLift.setPosition(0.8);
       }

       private void doRestStuff()
       {
           //************************
           // Lift claw and setup position
           sleep(500);
           clawLift.setPosition(1);
           sleep(2000);
           clawLeft.setPosition(0.5);
           clawRight.setPosition(1);
           lsRight.setPower(1);
           lsLeft.setPower(-1);
           sleep(250);
           lsRight.setPower(0);
           lsLeft.setPower(0);
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

       }
        private void setRunMode (DcMotor.RunMode mode){
            frontLeftMotor.setMode(mode);
            frontRightMotor.setMode(mode);
            rearLeftMotor.setMode(mode);
            rearRightMotor.setMode(mode);
        }

        private void moveDistance ( double inches, double power){
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



        private void strafeRight ( double inches, double power){
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

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition()+targetPosition);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() +targetPosition);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition()-targetPosition);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition()-targetPosition);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() &&rearRightMotor.isBusy()) {
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
        private int calculateTurnCountsLeft () {
            // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
            // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
            double robotWidth = 28; // This value represents half the distance between the wheels
            double wheelCircumference = Math.PI * robotWidth;
            double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
            return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
        }

        private int calculateTurnCountsRight () {
            // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
            // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
            double robotWidth = 27.5; // This value represents half the distance between the wheels
            double wheelCircumference = Math.PI * robotWidth;
            double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
            return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
        }

        private void stopRobot () {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);

            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private void resetEncoderCounts () {
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

        private void resetRobotPosition () {
            // Reset any other variables or mechanisms used for position tracking or orientation
            frontLeftMotorCounts = 0;
            frontRightMotorCounts = 0;

            rearLeftMotorCounts = 0;
            rearRightMotorCounts = 0;
            // For encoder-based position tracking, resetting the counts is sufficient in this example
        }
    }
