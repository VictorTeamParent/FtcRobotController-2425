package teamcode.OpenCVExt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name = "OpenCV1CamsExample")

public class OpenCV1CamsExample extends LinearOpMode {

    OpenCvWebcam webcam;
    RedConeLocDetection pipeline;
    RedConeLocDetection.RedConePosition position = RedConeLocDetection.RedConePosition.LEFT;
    //The following are for 2 cameras

    private RedConeLocDetection RedConeLocDetector;

    @Override
    public void runOpMode() throws InterruptedException {

        // the following is for one camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RedConeLocDetection();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //RedConeLocDetector = new RedConeLocDetection();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        // the above is for one camera
    }
}