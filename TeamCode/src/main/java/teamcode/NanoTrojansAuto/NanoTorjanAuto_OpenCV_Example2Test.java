package teamcode.NanoTrojansAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.OpenCVExt.LSideConeLocDetectionTest;


/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name = "Auto_RedRight_OpenCV_example2Test")
public class NanoTorjanAuto_OpenCV_Example2Test extends LinearOpMode {

    OpenCvWebcam webcam;
    LSideConeLocDetectionTest pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        // the following is for one camera
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        pipeline = new LSideConeLocDetectionTest();
        webcam.setPipeline(pipeline);


        /*
         * Start camera thread and setup camera resolutions. This resolution should be the same as what was set in the
         * location detection algorithm
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.addData("Analysis", pipeline.getPosition());
            telemetry.addData("Red Pix Detected", pipeline.redPixDetect());
            telemetry.addData("Blue Pix Detected", pipeline.bluePixDetect());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);


        }
    }

}
