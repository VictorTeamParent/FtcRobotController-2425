package teamcode.OpenCVExt;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import org.opencv.imgproc.Imgproc;

public class RedColorDetection {
    public static boolean containsRedColor(Mat inputImage) {
        // Convert the image from BGR to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(inputImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define the range of red color in HSV
//        Scalar lowerRed = new Scalar(0, 100, 100);
//        Scalar upperRed = new Scalar(10, 255, 255);

        Scalar lowerRed = new Scalar(254, 100, 100);
        Scalar upperRed = new Scalar(255, 255, 255);

        // Threshold the image to detect red color
        Mat redMask = new Mat();
        Core.inRange(hsvImage, lowerRed, upperRed, redMask);

        // Count the non-zero pixels in the red mask
        int nonZeroCount = Core.countNonZero(redMask);

        // Set a threshold for the number of red pixels to consider as red presence
        int threshold = 10; // Adjust as needed

        // Return true if the number of red pixels exceeds the threshold
        return nonZeroCount > threshold;
    }

    public static int redPixCount(Mat inputImage) {
        // Convert the image from BGR to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(inputImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define the range of red color in HSV
//        Scalar lowerRed = new Scalar(0, 100, 100);
//        Scalar upperRed = new Scalar(10, 255, 255);

        Scalar lowerRed = new Scalar(254, 100, 100);
        Scalar upperRed = new Scalar(255, 255, 255);

        // Threshold the image to detect red color
        Mat redMask = new Mat();
        Core.inRange(hsvImage, lowerRed, upperRed, redMask);

        // Count the non-zero pixels in the red mask
        int nonZeroCount = Core.countNonZero(redMask);

        // Set a threshold for the number of red pixels to consider as red presence
        int threshold = 10; // Adjust as needed

        // Return true if the number of red pixels exceeds the threshold
        return nonZeroCount ;
    }
}
