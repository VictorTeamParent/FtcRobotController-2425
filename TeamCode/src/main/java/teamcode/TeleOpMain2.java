//package org.firstinspires.ftc.teamcode;
package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Meacnum Control Drive V2", group="TeleOp")
public class TeleOpMain2 extends LinearOpMode {

    private DcMotor liftIntake = null;
    private DcMotor intake = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private final double driveAdjuster = 1;

    // the following are for huskylen
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode()  throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        liftIntake = hardwareMap.dcMotor.get("liftIntake");
        intake = hardwareMap.dcMotor.get("intake");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //********************** Husky Lens start *********************
        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();
        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        //huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        //********************** Husky Lens end *********************


        telemetry.update();
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        //while (!isStopRequested()) {
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            final double globalAngle = angles.firstAngle;

            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            final double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI /4;
            final double rightX = gamepad1.right_stick_y;

            double v1 = r * Math.sin(robotAngle - globalAngle/57) - rightX;
            double v2 = r * Math.cos(robotAngle - globalAngle/57) + rightX;
            double v3 = r * Math.cos(robotAngle - globalAngle/57) - rightX;
            double v4 = r * Math.sin(robotAngle - globalAngle/57) + rightX;

            if (Math.abs(v1) > 1 || Math.abs(v2) > 1 || Math.abs(v3) > 1 || Math.abs(v4) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(v1), Math.abs(v2));
                max = Math.max(Math.abs(v3), max);
                max = Math.max(Math.abs(v4), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                v1 /= max;
                v2 /= max;
                v3 /= max;
                v4 /= max;
            }

            frontRight.setPower(v1);
            frontLeft.setPower(v2);
            backRight.setPower(v3);
            backLeft.setPower(v4);

            telemetry.addData("Heading ", globalAngle);
            telemetry.addData("Stick1 ", robotAngle);

            // Get inpurt from gamepad 2 and use the value to control intake motor and
            // liftIntak motor
            liftIntake.setPower(gamepad2.left_stick_y);
            intake.setPower(gamepad2.right_stick_y);

            // the following liens print out log information in the driver screen
            telemetry.addData("Gamepad2_left_y ", gamepad2.left_stick_y);
            telemetry.addData("Gamepad2_right_y ", gamepad2.right_stick_y);

            //********************** Husky Lens start *********************
            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            //********************** Husky Lens end *********************


            telemetry.update();

        }

//        while (opModeIsActive()) {
//
//
//
//        }
    }
}
