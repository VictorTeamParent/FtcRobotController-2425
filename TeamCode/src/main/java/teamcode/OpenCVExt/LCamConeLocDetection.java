package teamcode.OpenCVExt;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//import teamcode.SkystoneDeterminationExample;


public class LCamConeLocDetection extends OpenCvPipeline {
    /*
     * An enum to define the skystone position
     */
    public enum LSideConePosition {
        LEFT,
        CENTER,
        RIGHT,
        OTHER
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Scalar RED = new Scalar(255, 0, 0);

    /*
     * The core values which define the location and size of the sample regions
     * The following is design for size 320 x 240 resolution
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(75, 0);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(255, 0);

//    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70, 0);
//    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(225, 0);

    //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(110, 75);
    //    //blue Left
    //static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(0, 85);

    static final int REGION_WIDTH = 65;
    static final int REGION_HEIGHT = 20;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region3_Cb;


    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile LSideConePosition position = LSideConePosition.OTHER;

     @Override
    public Mat processFrame(Mat input) {

        region1_Cb = input.submat(new Rect(region1_pointA, region1_pointB));
        region3_Cb = input.submat(new Rect(region3_pointA, region3_pointB));

        Scalar sumColorsRg1 = Core.sumElems(region1_Cb);
        Scalar sumColorsRg3 = Core.sumElems(region3_Cb);

        double maxColorR1 = Math.max(sumColorsRg1.val[0], Math.max(sumColorsRg1.val[1], sumColorsRg1.val[2]));
        double maxColorR3 = Math.max(sumColorsRg3.val[0], Math.max(sumColorsRg3.val[1], sumColorsRg3.val[2]));

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

         /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        if(sumColorsRg1.val[0] == maxColorR1 || sumColorsRg1.val[2] == maxColorR1)
        {
            position = LSideConePosition.LEFT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
       } //else if (max == avg3) // Was it from region 3?
        else if (sumColorsRg3.val[0] == maxColorR3 || sumColorsRg3.val[2] == maxColorR3)
        {
            position = LSideConePosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else
            position = LSideConePosition.RIGHT; // Record our analysis

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public LSideConePosition getPosition() {
        return position;
    }
}