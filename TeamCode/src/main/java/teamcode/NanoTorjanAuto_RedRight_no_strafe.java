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

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="Auto_RedRightNoStrafe")
public class NanoTorjanAuto_RedRight_no_strafe extends LinearOpMode
{
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    static final double COUNTS_PER_INCH = 537.7; // Replace this value with your actual counts per inch
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize motors
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        // Set motor directions
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//
//        // Set motor modes
//        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        waitForStart();
//
//        // Move the robot forward 12 inches
//        moveDistance(2);
//
//        // Stop the robot
//        stopRobot();
//    }
//
//    private void setRunMode(DcMotor.RunMode mode) {
//        frontLeft.setMode(mode);
//        frontRight.setMode(mode);
//        backLeft.setMode(mode);
//        backRight.setMode(mode);
//    }
//
//    private void moveDistance(double inches) {
//        int targetPosition = (int) (inches * COUNTS_PER_INCH);
//
//        frontLeft.setTargetPosition(targetPosition);
//        frontRight.setTargetPosition(targetPosition);
//        backLeft.setTargetPosition(targetPosition);
//        backRight.setTargetPosition(targetPosition);
//
//        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        double power = .5; // Adjust power as needed
//        frontLeft.setPower(power);
//        frontRight.setPower(power);
//        backLeft.setPower(power);
//        backRight.setPower(power);
//
//        while (opModeIsActive() &&
//                frontLeft.isBusy() &&
//                frontRight.isBusy() &&
//                backLeft.isBusy() &&
//                backRight.isBusy()) {
//            // Wait for motors to reach target position
//        }
//
//        stopRobot();
//    }
//
//    private void stopRobot() {
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//
//        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }


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
    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch


    private int frontLeftMotorCounts = 0;
    private int frontRightMotorCounts = 0;

    private int rearLeftMotorCounts = 0;
    private int rearRightMotorCounts = 0;

    @Override
    public void runOpMode() {
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

        waitForStart();

        //moveBackward(3);
        //sleep(500); // milliseconds

        //while(opModeIsActive()) {
            // Move the robot forward 12 inches
            sleep(250);
            clawLeft.setPosition(1);
            clawRight.setPosition(0.5);
            moveDistance(25, 0.3);

            // Pause for a brief moment (adjust as needed)


            sleep(1000); // milliseconds

           //moveDistance(-5, 0.3);

            // Perform a 90-degree right turn
            turnRight90D(1);

            // Pause for a brief moment (adjust as needed)
            sleep(500); // milliseconds

            // Perform a 90-degree right turn
            //turnLeft90D();

             moveDistance(30, 0.4);
             sleep(1000);
        moveDistance(4, 0.2);
        sleep(1000);
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


//            sleep(250);
//            strafeRight(22, 1);
//
//            sleep(250);
//            moveDistance(12, 0.3);


        // Stop the robot
            stopRobot();
        //}
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }

    private void moveDistance(double inches , double power) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition()+targetPosition);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition()+targetPosition);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition()+targetPosition);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition()+targetPosition);


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

    private void strafeRight(double inches, double power) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition()-targetPosition);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() -targetPosition);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition()+targetPosition);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition()+targetPosition);

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
    private int calculateTurnCountsLeft() {
        // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
        // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
        double robotWidth = 28; // This value represents half the distance between the wheels
        double wheelCircumference = Math.PI * robotWidth;
        double countsPerInch = COUNTS_PER_INCH; // Use your previously calculated value
        return (int) ((wheelCircumference / 4.0) * countsPerInch); // 90-degree turn for each wheel
    }

    private int calculateTurnCountsRight() {
        // Calculate encoder counts needed for a 90-degree turn based on robot-specific measurements
        // Example calculation: Assume each motor needs to move half of the circumference of a circle with a 12-inch radius
        double robotWidth = 27.5; // This value represents half the distance between the wheels
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
