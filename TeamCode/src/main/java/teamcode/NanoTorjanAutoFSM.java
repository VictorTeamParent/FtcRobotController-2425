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

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="NanoTorjanAutoFSM")

public class NanoTorjanAutoFSM extends LinearOpMode {

    // Define motor constants
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0;      // Diameter of the wheel in millimeters
    static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_MM);

    // Define target distances in inches
    static final double FORWARD_DISTANCE_INCHES = 10.0;
    static final double BACKWARD_DISTANCE_INCHES = 3.0;

    // Define target angles for turns
    static final int TURN_90_DEGREES = 90;

    // Enum to represent different states
    enum RobotState {
        INIT,
        MOVE_FORWARD,
        CHECK_FORWARD_ENCODER,
        TURN_90_DEGREES,
        CHECK_TURN_ENCODER,
        MOVE_BACKWARD,
        CHECK_BACKWARD_ENCODER,
        STOP
    }

    // Motors
    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;

    @Override
    public void runOpMode() {
        // Initialize motors and encoders
        leftFrontMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackMotor = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Reset encoders
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set encoder run mode
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial state
        RobotState currentState = RobotState.INIT;

        waitForStart();

        while (opModeIsActive()) {
            // Finite State Machine
            switch (currentState) {
                case INIT:
                    currentState = RobotState.MOVE_FORWARD;
                    break;

                case MOVE_FORWARD:
                    int forwardTargetCounts = (int) (FORWARD_DISTANCE_INCHES * COUNTS_PER_MM);

                    setTargetPositionForAllMotors(forwardTargetCounts);
                    setMotorsRunToPosition();
                    setMotorsPower(0.5);

                    currentState = RobotState.CHECK_FORWARD_ENCODER;
                    break;

                case CHECK_FORWARD_ENCODER:
                    if (!areMotorsBusy()) {
                        setMotorsPower(0);
                        currentState = RobotState.TURN_90_DEGREES;
                    }
                    break;

                case TURN_90_DEGREES:
                    int turnTargetCounts = (int) (TURN_90_DEGREES * COUNTS_PER_MM);

                    setTargetPositionForAllMotors(turnTargetCounts, -turnTargetCounts);
                    setMotorsRunToPosition();
                    setMotorsPower(0.5);

                    currentState = RobotState.CHECK_TURN_ENCODER;
                    break;

                case CHECK_TURN_ENCODER:
                    if (!areMotorsBusy()) {

                        setMotorsPower(0);
                        resetMotorsRunMode();
                        currentState = RobotState.MOVE_BACKWARD;
                    }
                    break;

                case MOVE_BACKWARD:
                    int backwardTargetCounts = (int) (BACKWARD_DISTANCE_INCHES * COUNTS_PER_MM);

                    setTargetPositionForAllMotors(-backwardTargetCounts);
                    setMotorsRunToPosition();
                    setMotorsPower(0.5);

                    currentState = RobotState.CHECK_BACKWARD_ENCODER;
                    break;

                case CHECK_BACKWARD_ENCODER:
                    if (!areMotorsBusy()) {
                        setMotorsPower(0);
                        currentState = RobotState.STOP;
                    }
                    break;

                case STOP:
                    // Stop the robot or perform any additional actions
                    // Transition to INIT or another state if needed
                    break;
            }

            telemetry.addData("Left Front Encoder", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Right Front Encoder", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Left Back Encoder", leftBackMotor.getCurrentPosition());
            telemetry.addData("Right Back Encoder", rightBackMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void resetMotorsRunMode() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    private void setTargetPositionForAllMotors(int... targetPositions) {
        if (targetPositions.length == 4) {
            leftFrontMotor.setTargetPosition(targetPositions[0]);
            rightFrontMotor.setTargetPosition(targetPositions[1]);
            leftBackMotor.setTargetPosition(targetPositions[2]);
            rightBackMotor.setTargetPosition(targetPositions[3]);
        }
    }

    private void setMotorsRunToPosition() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorsPower(double power) {
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
    }

    private boolean areMotorsBusy() {
        return leftFrontMotor.isBusy() || rightFrontMotor.isBusy() ||
                leftBackMotor.isBusy() || rightBackMotor.isBusy();
    }
}
