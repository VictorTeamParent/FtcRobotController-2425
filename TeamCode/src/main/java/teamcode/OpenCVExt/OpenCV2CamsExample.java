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
@Autonomous(name = "OpenCV2CamsExample")

public class OpenCV2CamsExample extends LinearOpMode {

    OpenCvWebcam webcam;
    RedConeLocDetection pipeline;
    RedConeLocDetection.RedConePosition position = RedConeLocDetection.RedConePosition.LEFT;
    //The following are for 2 cameras
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvWebcam openCvWebCamera1;
    OpenCvWebcam openCvWebCamera2;
    RedConeLocDetection pipeline1;
    RedConeLocDetection pipeline2;
    OpenCvSwitchableWebcam switchableWebcam;
    private RedConeLocDetection RedConeLocDetector;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         *  Get 2 web camera name
         */

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        OpenCvCamera openCvWebCamera1 = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
        OpenCvCamera openCvWebCamera2 = OpenCvCameraFactory.getInstance().createWebcam(webcam2);
        /**
         * create switchablewebcam
         */
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
        pipeline1 = new RedConeLocDetection();
        pipeline2 = new RedConeLocDetection();

        /*
         *  The following pipeline is not streaming, need to find out why
         */
        openCvWebCamera1.setPipeline(pipeline1);
        openCvWebCamera2.setPipeline(pipeline2);
        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addLine("Switch to camera 1\n");
            switchableWebcam.setActiveCamera(webcam1);
            //openCvWebCamera1.setPipeline(pipeline1);
            telemetry.addLine("Switch to camera 1\n");
            sleep(3000);

            switchableWebcam.setActiveCamera(webcam2);
            //openCvWebCamera2.setPipeline(pipeline2);
            telemetry.addLine("Switch to camera 2\n");
            sleep(3000);

        }
    }
}