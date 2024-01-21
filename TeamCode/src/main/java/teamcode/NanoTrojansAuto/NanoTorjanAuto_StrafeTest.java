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
//import teamcode.drive.SampleMecanumDrive;
import teamcode.trajectorysequence.TrajectorySequence;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name = "Auto_Strafe_test")
public class NanoTorjanAuto_StrafeTest extends LinearOpMode {

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
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontLeft");

//        rearLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
//        rearRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        rearLeftMotor = hardwareMap.get(DcMotor.class, "backRight");
        rearRightMotor = hardwareMap.get(DcMotor.class, "backLeft");

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

        waitForStart();
//        drive.followTrajectory(trajectory);
        boolean stop = false;
        while (opModeIsActive() && !stop) {

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            Run2seconds (3, 0.2);
                stop = true;
            }

        }

    private void Run2seconds ( double inches, double power){


        frontRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        rearRightMotor.setPower(power);
        rearLeftMotor.setPower(power);


        sleep(3000);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);


    }


    }
