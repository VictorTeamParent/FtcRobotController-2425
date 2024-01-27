package teamcode.NanoTrojansAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.LSideConeLocDetection;
import teamcode.OpenCVExt.RedConeLocDetection;


/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name = "Auto_RedRight_OpenCV_example2")
public class NanoTorjanAuto_OpenCV_Example2 extends LinearOpMode {

    OpenCvWebcam webcam2;
    LSideConeLocDetection pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        // the following is for one camera
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        pipeline = new LSideConeLocDetection();
        webcam2.setPipeline(pipeline);


        /*
         * Start camera thread and setup camera resolutions. This resolution should be the same as what was set in the
         * location detection algorithm
         */
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //put your code to handle error messages
            }
        });


        waitForStart();

        /*
         * the following loop will keep running when opModeIsActive is true.
         * This is actually a thread
         */
        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            if (pipeline.getPosition() == LSideConeLocDetection.LSideConePosition.CENTER) {

                // moveDistance(10, 0.3);
                telemetry.addLine("Move forward 10 inches");

            }
        }
    }

}
