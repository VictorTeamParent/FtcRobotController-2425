package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveControl_NanoTorjan {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Orientation angles;

    public DriveControl_NanoTorjan(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, BNO055IMU imuInstance) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
        this.imu = imuInstance;
    }

    public void driveRobot(double leftStickX, double leftStickY, double rightStickX) {
        double y = -leftStickY; // Remember, this is reversed!
        double x = leftStickX * 1.1; // Counteract imperfect strafing
        double rx = rightStickX;   // Right Stick X value is used to control rotation of the roberts

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        //Victor: max value for all the settings, Left Y, left X and Right x
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        // Here are possible scenarios
        // for wheel moving direction follow the following link
        // https://www.gobilda.com/strafer-chassis-kit-140mm-mecanum-wheels/
        // 1. Only move forward and backward
        //    then x and rx value is 0
        // 2. only Strafing
        //    then y and rx will be 0

        double frontLeftPower = (y - x + rx) / denominator;
        double backLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}