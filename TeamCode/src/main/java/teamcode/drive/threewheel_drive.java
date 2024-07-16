package teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "threewheeldrive", group = "TeleOp")

public class threewheel_drive extends LinearOpMode{
    private DcMotor Left;
    private DcMotor Right;
    @Override
    public void runOpMode() throws InterruptedException {


        double power = 0;
        double turnl = 0;
        double turnr = 0;
        double LeftPower;
        double RightPower;

        power = gamepad1.right_stick_y;
        Right.setPower(power);
        Left.setPower(-power);

        turnl = gamepad1.left_stick_x;
        Left.setPower(turnl);

        turnr = -gamepad1.left_stick_x;
        Right.setPower(turnr);


//        elif (gamepad1.right_stick_x){
//            Left.setPower(1);
//        }
    }


}