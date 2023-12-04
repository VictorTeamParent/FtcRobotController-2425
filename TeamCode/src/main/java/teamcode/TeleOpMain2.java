package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Meacnum Control Drive V2", group="TeleOp")
public class TeleOpMain2 extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private final double driveAdjuster = 1;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode()  throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.update();
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            final double globalAngle = angles.firstAngle;

            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            final double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI /4;
            final double rightX = gamepad1.right_stick_x;

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
            telemetry.addData("Stick ", robotAngle);
            telemetry.update();
        }
    }
}
