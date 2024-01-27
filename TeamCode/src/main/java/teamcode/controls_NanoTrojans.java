package teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



public class controls_NanoTrojans  {
    private DcMotor intake = null;
    private DcMotor lsRight = null;
    private DcMotor lsLeft = null;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //servo motors
    private CRServo planeLaunch = null;

    //2 claws servo motors
    private Servo clawLeft = null;
    private Servo clawRight = null;

    //2 arms servo motors
    private Servo clawLift = null;
    private Servo armLift = null;
    private CRServo robotLift = null;
    public controls_NanoTrojans(DcMotor intak, DcMotor lsR, DcMotor lsL, CRServo planeL,
                                     Servo clawL, Servo clawR, Servo clawLi, Servo armL, CRServo robotL)
    {
        intake= intak;
        lsRight=lsR;
        lsLeft=lsL;
        planeLaunch=planeL;
        clawLeft=clawL;
        clawRight=clawR;
        clawLift=clawLi;
        armLift=armL;
        robotLift=robotL;
    }
    public void closeClaw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        clawLeft.setPosition(1);
        clawRight.setPosition(0.5);
    }
    public void openClaw()
    {
        //for the claw, it is a regular motor so you set positions; you just have to keep tweaking the code and test out positions that you input.
        clawLeft.setPosition(0.5);
        clawRight.setPosition(1);
    }
    public void hangSpin()
    {
        //since this servo is continuous, we have to use set power like motors; then sleep 1000 milliseconds which is equal to one second before turning the servo off.
        //you can also hold down the button to continuously turn the servo instead of pressing multiple times.
        robotLift.setPower(1);
    }
    public void hangSpinstop()
    {
        robotLift.setPower(0);
    }
    public void reversehangSpin()
    {
        // this is the same as the hang spin except reversing the servo so it goes the other way so that we can unload the tension.
        robotLift.setPower(-1);
    }
    public void reversehangSpinstop()
    {
        robotLift.setPower(0);
    }
    public void planeLaunch()
    {
        //plane launch also uses a continuous servo so it has the same concept as the hang mechanism, it basically just turns the servo for a second and then turns it off.
        planeLaunch.setPower(1);
    }
    public void planeLaunchstop()
    {
        planeLaunch.setPower(0);
    }
    public void clawDown()
    {
        //these follow the same concept as the claw, except it only needs to move one servo.
        clawLift.setPosition(0.61);
    }
    public void clawUp()
    {
        //same thing for this except the position is different.
        clawLift.setPosition(0.8);
    }
    public void clawFull()
    {
        clawLift.setPosition(1);
    }
    public void armUp()
    {
        //same concept as clawUp, just on the arm.
        armLift.setPosition(0.7);
    }
    public void armDown()
    {
        //same thing as armUp but with a different position.
        armLift.setPosition(0.125);
    }
    public void armFull()
    {
        armLift.setPosition(0.8);
    }
    public void smallls()
    {
        //since this is a motor, it uses power, this works the same as robotlift but one of the powers are negative because one of the motors is facing the opposite way.
        //since this is the small linear slide lift, we only wait 250 milliseconds which is equivilant to a quarter of a second; then we just turn off the motors.
        lsRight.setPower(-1);
        lsLeft.setPower(1);
    }
    public void smalllsstop()
    {
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }
    public void reversesmallls()
    {
        lsRight.setPower(1);
        lsLeft.setPower(-1);
    }
    public void reversesmalllsstop()
    {
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }
    public void mediumls()
    {
        //same as smallls but let the motor run for longer.
        lsRight.setPower(-1);
        lsLeft.setPower(1);
    }
    public void mediumlsstop()
    {
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }
    public void reversemediumls()
    {
        lsRight.setPower(1);
        lsLeft.setPower(-1);
    }
    public void reversemediumlsstop()
    {
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }

    public void highls()
    {
        //same as mediumls but let motor run for even longer.
        //this is only raising it from medium by a little bit because we wont go any higher than that within the time limit.
        lsRight.setPower(-1);
        lsLeft.setPower(1);
    }
    public void highlsstop()
    {
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }
    public void reversehighls()
    {
        lsRight.setPower(1);
        lsLeft.setPower(-1);
    }
    public void reversehighlsstop()
    {
        lsRight.setPower(0);
        lsLeft.setPower(0);
    }
}