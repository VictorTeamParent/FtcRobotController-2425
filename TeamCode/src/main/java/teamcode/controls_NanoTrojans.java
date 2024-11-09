package teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class controls_NanoTrojans {
    // Constants for encoder counts and wheel measurements
    static final double COUNTS_PER_REVOLUTION = 537.7; // Encoder counts per revolution
    static final double WHEEL_DIAMETER_MM = 96.0; // Wheel diameter in millimeters
    static final double MM_PER_REVOLUTION = WHEEL_DIAMETER_MM * Math.PI; // Wheel circumference
    static final double COUNTS_PER_MM = COUNTS_PER_REVOLUTION / MM_PER_REVOLUTION; // Counts per millimeter
    static final double COUNTS_PER_INCH = COUNTS_PER_MM * 25.4; // Counts per inch

    //private DcMotor intake = null;
    private DcMotor lsRight = null;
    private DcMotor lsLeft = null;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //servo motors
    private Servo rhsl = null;

    //2 claws servo motors
    private Servo lhsl = null;
    private Servo claw = null;

    //2 arms servo motors
    private CRServo intakewheels = null;
    private Servo intakelift = null;
    private Servo clawlift = null;
    private Servo casket = null;
    //private CRServo robotLift = null;

    //Victor for auto pick

    public boolean autopicksucess = false;
    public double hls = 0;
    public double activeintake = 0;





    public controls_NanoTrojans(DcMotor lsR, DcMotor lsL, Servo lhorizontal,
                                Servo rhorizontal, Servo claw, Servo clawlift, Servo intakelift, CRServo intakewheels, Servo casket)
    {
        //intake= intak;
        lsRight=lsR;
        lsLeft=lsL;
        lhorizontal=lhsl;
        rhorizontal=rhsl;
        claw =claw;
        clawlift=clawlift;
        intakelift =intakelift;
        //robotLift=robotL;
//        dcArm =arm;
    }
    public void closeclaw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        claw.setPosition(0.55);
    }
    public void openclaw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        claw.setPosition(0.8);

    }

    public void horizontal_fw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.

        lhsl.setPosition(0.5);
        rhsl.setPosition(0.5);
    }

    public void horizontal_back()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        lhsl.setPosition(0.7);
        rhsl.setPosition(0.3);
    }
    public void casket_fw()
    {
        casket.setPosition(0.5);
    }
    public void casket_back()
    {
        casket.setPosition(0.7);
    }

//    public void intake(){
//        intake
//    }


//    public void openLeftClaw()
//    {
//        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
//        clawLeft.setPosition(0.5);
//    }
//    public void openLeftClawWide()
//    {
//        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
//        clawLeft.setPosition(0.4);
//    }
//
//    public void openRightClaw()
//    {
//        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
//        claw.setPosition(1);
//
//    }

//    public void hangSpin()
//    {
//        //since this servo is continuous, we have to use set power like motors; then sleep 1000 milliseconds which is equal to one second before turning the servo off.
//        //you can also hold down the button to continuously turn the servo instead of pressing multiple times.
//        robotLift.setPower(1);
//    }
//    public void hangSpinstop()
//    {
//        robotLift.setPower(0);
//    }
//    public void reversehangSpin()
//    {
//        // this is the same as the hang spin except reversing the servo so it goes the other way so that we can unload the tension.
//        robotLift.setPower(-1);
//    }
//    public void reversehangSpinstop()
//    {
//        robotLift.setPower(0);
//    }
//    public void planeLaunch()
//    {
//        //plane launch also uses a continuous servo so it has the same concept as the hang mechanism, it basically just turns the servo for a second and then turns it off.
//        planeLaunch.setPower(-0.2);
//        //planeLaunch.setPosition(0.55);
//    }
//    public void planeLaunchstop()
//    {
//        planeLaunch.setPower(0);
//        //planeLaunch.setPosition(0.5);
//    }
//    public void clawDown()
//    {
        //these follow the same concept as the claw, except it only needs to move one servo.
//        clawLift.setPosition(0.625);
        //clawLift.setPosition(0.395);
//        clawLift.setPosition(0.380);
//        clawdown = true;
//
//    }

//      public void clawUp()
//    {
//        //same thing for this except the position is different.
////        clawLift.setPosition(0.8);
//        clawLift.setPosition(0.75);
//        clawdown = false;
//        autopicksucess = false;
//
//    }

//    public void clawUpParallerToBoard()
//    {
//        //same thing for this except the position is different.
////        clawLift.setPosition(0.8);
//        clawLift.setPosition(0.90);
//        clawdown = false;
//        autopicksucess = false;
//
//    }
//    public void clawFull()
//    {
//
//        clawLift.setPosition(0.75);
//        clawdown = false;
//    }
//    public void clawparallel()
//    {
//        clawLift.setPosition(0.69);
//    }
//
//    public void armUp()
//    {
//        //same concept as clawUp, just on the arm.
//        intakelift.setPosition(0.25);
////        moveArm(6,1);
////        armLift.setPosition(0);
//    }
//    public void armDown()
//    {
//        //same thing as armUp but with a different position.
//        //armLift.setPosition(0.085);
//        intakelift.setPosition(0.045);
////        moveArm(-6,1);
//    }
//    public void armFull()
//    {
//        intakelift.setPosition(0.95);
//    }
//    public void smallls()
//    {
//        //since this is a motor, it uses power, this works the same as robotlift but one of the powers are negative because one of the motors is facing the opposite way.
//        //since this is the small linear slide lift, we only wait 250 milliseconds which is equivilant to a quarter of a second; then we just turn off the motors.
//        lsRight.setPower(-1);
//        lsLeft.setPower(1);
//    }
//    public void smalllsstop()
//    {
//        lsRight.setPower(0);
//        lsLeft.setPower(0);
//    }
//    public void reversesmallls()
//    {
//        lsRight.setPower(1);
//        lsLeft.setPower(-1);
//    }
//    public void reversesmalllsstop()
//    {
//        lsRight.setPower(0);
//        lsLeft.setPower(0);
//    }
//    public void mediumls()
//    {
//        //since this is a motor, it uses power, this works the same as robotlift but one of the powers are negative because one of the motors is facing the opposite way.
//        //since this is the small linear slide lift, we only wait 250 milliseconds which is equivilant to a quarter of a second; then we just turn off the motors.
//        lsRight.setPower(-1);
//        lsLeft.setPower(1);
//    }
//    public void mediumlsstop()
//    {
//        lsRight.setPower(0);
//        lsLeft.setPower(0);
//    }
//    public void reversemediumls()
//    {
//        lsRight.setPower(1);
//        lsLeft.setPower(-1);
//    }
//    public void reversemediumlsstop()
//    {
//        lsRight.setPower(0);
//        lsLeft.setPower(0);
//    }
//    public void highls()
//    {
//        //since this is a motor, it uses power, this works the same as robotlift but one of the powers are negative because one of the motors is facing the opposite way.
//        //since this is the small linear slide lift, we only wait 250 milliseconds which is equivilant to a quarter of a second; then we just turn off the motors.
//        lsRight.setPower(-1);
//        lsLeft.setPower(1);
//    }
//    public void highlsstop()
//    {
//        lsRight.setPower(0);
//        lsLeft.setPower(0);
//    }
//    public void reversehighls()
//    {
//        lsRight.setPower(1);
//        lsLeft.setPower(-1);
//    }
//    public void reversehighlsstop()
//    {
//        lsRight.setPower(0);
//        lsLeft.setPower(0);
//    }


//    public boolean detectPixel (ColorSensor cs)
//    {
//        boolean rc = false;
//        boolean enableTelemetry = false;
//        int red = cs.red();
//        int green = cs.green();
//        int blue = cs.blue();
//
//        //boolean rightpixeldetected = false;
//        if(enableTelemetry) {
//            telemetry.addData("Red", red);
//            telemetry.addData("Green", green);
//            telemetry.addData("Blue", blue);
//            telemetry.update();
//        }
//
//        // Check for green color
//        //else if (green > 250 && red < 200 && blue > 200) {
//        if ( red > 150 && green > 250 && blue > 200) {
//            if(enableTelemetry) {
//                telemetry.addData("Color", "Green");
//            }
//            rc = true;
//        }
//        // Check for white color
//        else if (red > 200 && green > 200 && blue > 200) {
//            if(enableTelemetry) {
//                telemetry.addData("Color", "White");
//            }
//            rc = true;
//        }
//        // None of the specified colors detected
//        else {
//            if(enableTelemetry) {
//                telemetry.addData("Color", "Unknown");
//            }
//        }
//
//        telemetry.update();
//        return rc;
//    }

//    private void moveArm(double inches, double power) {
//        int targetPosition = (int) (inches * COUNTS_PER_INCH);
//
//        dcArm.setTargetPosition(dcArm.getCurrentPosition() + targetPosition);
//
//        dcArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        //double power = 0.3; // Adjust power as needed
//        dcArm.setPower(power);
//
//        while ( dcArm.isBusy()) {
//            // Wait for motors to reach target position
//        }

//        resetEncoderCounts();
//        resetRobotPosition();
//        stopRobot();
//    }
}