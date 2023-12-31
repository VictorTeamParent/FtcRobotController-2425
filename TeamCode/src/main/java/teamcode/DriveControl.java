package teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveControl {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Orientation angles;

    public DriveControl(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, BNO055IMU imuInstance) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
        this.imu = imuInstance;
    }

    public void driveRobot(double leftStickX, double leftStickY, double rightStickX) {
        // Get robot's heading from IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        final double globalAngle = angles.firstAngle;

        // Calculate drive power for each motor based on gamepad inputs
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(-leftStickY, leftStickX) - Math.PI / 4;
        double v1 = r * Math.sin(robotAngle - globalAngle * (Math.PI / 180)) + rightStickX;
        double v2 = r * Math.cos(robotAngle - globalAngle * (Math.PI / 180)) - rightStickX;
        double v3 = r * Math.cos(robotAngle - globalAngle * (Math.PI / 180)) + rightStickX;
        double v4 = r * Math.sin(robotAngle - globalAngle * (Math.PI / 180)) - rightStickX;

        // Normalize motor powers
        double max = Math.max(Math.abs(v1), Math.abs(v2));
        max = Math.max(max, Math.abs(v3));
        max = Math.max(max, Math.abs(v4));
        if (max > 1) {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }

        // Set power to motors
        frontLeft.setPower(v1);
        frontRight.setPower(-v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);

        telemetry.addData("Heading ", globalAngle);
        telemetry.addData("Stick1 ", robotAngle);
    }
}