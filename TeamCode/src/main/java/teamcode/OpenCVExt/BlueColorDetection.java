package teamcode.OpenCVExt;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class BlueColorDetection {
    public static boolean containsBlueColor(Mat inputImage) {
        // Convert the image from BGR to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(inputImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define the range of blue color in HSV
        Scalar lowerBlue = new Scalar(0, 0, 245);
        Scalar upperBlue = new Scalar(10, 10, 255);

        // Threshold the image to detect blue color
        Mat blueMask = new Mat();
        Core.inRange(hsvImage, lowerBlue, upperBlue, blueMask);

        // Count the non-zero pixels in the blue mask
        int nonZeroCount = Core.countNonZero(blueMask);

        // Set a threshold for the number of blue pixels to consider as blue presence
        int threshold = 100; // Adjust as needed

        // Return true if the number of blue pixels exceeds the threshold
        return nonZeroCount > threshold;
    }
}

