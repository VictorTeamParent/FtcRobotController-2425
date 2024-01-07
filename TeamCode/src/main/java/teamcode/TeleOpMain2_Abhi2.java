package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name= "TeleOp-Abhi2" , group="TeleOp")
public class TeleOpMain2_Abhi2 extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    private DriveControl driveControl;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor. get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create parameters object and configure IMU parameters
        Parameters parameters = new Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // ... (other configurations if needed)

        imu.initialize(parameters);


        driveControl = new DriveControl(frontLeft, frontRight, backLeft, backRight, imu);

        waitForStart();
        while  (opModeIsActive()) {
            /*double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;
            double rightStickY= gamepad1.right_stick_x;

            driveControl.driveRobot(leftStickX, leftStickY, rightStickY);*/
            double leftStickY = -gamepad1.left_stick_y;  // Use left joystick Y-axis for forward/backward movement
            double leftStickX = gamepad1.left_stick_x;   // Use left joystick X-axis for strafing

            double rightStickX = gamepad1.right_stick_x; // Use right joystick X-axis for turning

            // Calculate motor powers based on joystick inputs
            double frontLeftPower = leftStickY - leftStickX + rightStickX;
            double frontRightPower = leftStickY + leftStickX - rightStickX;
            double backLeftPower = leftStickY + leftStickX + rightStickX;
            double backRightPower = leftStickY - leftStickX - rightStickX;

            // Normalize motor powers
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Set calculated powers to the motors
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            telemetry.addData("LeftStickX", leftStickX);
            telemetry.addData("LeftStickY", leftStickY);
            telemetry.addData("RightStickX", rightStickX);
            telemetry.update();


        }

    }
}