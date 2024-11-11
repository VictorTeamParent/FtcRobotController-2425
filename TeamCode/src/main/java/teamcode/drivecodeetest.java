package teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import teamcode.drive.SampleMecanumDrive;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@TeleOp(name = "drivecode", group = "TeleOp")

public class drivecodeetest extends LinearOpMode {
    private DriveControl_NanoTorjan driveControl;
    //private DriveControl driveControl;

    private controls_NanoTrojans g2control;

    private resources_NanoTrojans resources;
    @Override
    public void runOpMode()  throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

        }
        while (opModeIsActive()) {
            driveControl.driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }}






