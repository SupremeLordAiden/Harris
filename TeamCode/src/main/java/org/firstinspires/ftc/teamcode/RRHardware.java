package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RRHardware {
    /* Public OpMode members. */

    //intake motors
    public DcMotor Squishy1         = null;
    public DcMotor Squishy2         = null;



    //Linear Slide Motor (ofc)
    public DcMotor LineraSlide      = null;




    //servos
    public Servo swipeServo         = null;

    //the thing that grabs the cube
    public Servo armthingy          = null;

    //the thing that moves the entire thing (chain lift)
    public Servo grabbythingy       = null;

    //foundation grabber
    public Servo foundationgrabber  = null;

    //capstone droper
    public Servo capstonedropper    = null;

    //bigger arm for auto
    public Servo autoPush           = null;

    //the arm for actually grabbing auto
    public Servo autoArm            = null;


    //bigger arm for auto left
    public Servo autoPush2          = null;

    //the arm for actually grabbing auto left
    public Servo autoArm2           = null;

    public ServoImplEx rgb                = null;

    //servo that deploys tape measure
    public CRServo tapeMeasure      = null;

    //Linera Slide Touch Sensor
    public DigitalChannel TouchSense   = null;

    //the color sensor
    public DistanceSensor sensorDistance = null;


    public DistanceSensor distance;

    public DistanceSensor distanceRight;

    public DistanceSensor distanceLeft;


    /* local OpMode members. */
    HardwareMap hwMap          = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RRHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;



        /*
        the 2 intake wheels are GoBilda 223 RPM motors with a rattio of 26.9:1
        These are 188.3 Encoder Cycles Per Revolution
         */
        Squishy1 = hwMap.get(DcMotor.class, "squishy1");
        Squishy2 = hwMap.get(DcMotor.class, "squishy2");




        //tetrix motor 100 rpm direct drive
        LineraSlide = hwMap.get(DcMotor.class, "LineraSlide");



        //touch sensor
        TouchSense = hwMap.get(DigitalChannel.class, "TouchSense");

        TouchSense.setMode(DigitalChannel.Mode.INPUT);




        //color sensor
        sensorDistance  = hwMap.get(DistanceSensor.class, "color");

        //intake distance sensor
        distance        = hwMap.get(DistanceSensor.class, "distance");
        distanceRight   = hwMap.get(DistanceSensor.class, "distanceright");
        distanceLeft    = hwMap.get(DistanceSensor.class, "distanceleft");




        //Linear Slide
        LineraSlide.setDirection(DcMotor.Direction.FORWARD);





        // Define and initialize ALL installed servos.
        armthingy           = hwMap.get(Servo.class, "armthingy");

        grabbythingy        = hwMap.get(Servo.class, "grabbythingy");

        swipeServo          = hwMap.get(Servo.class, "swipe");

        foundationgrabber   = hwMap.get(Servo.class, "foundationgrabber");

        capstonedropper     = hwMap.get(Servo.class, "dropper");

        autoArm             = hwMap.get(Servo.class, "autoarm");

        autoPush            = hwMap.get(Servo.class, "autopush");

        tapeMeasure         = hwMap.get(CRServo.class, "tapeMeasure");

        autoArm2            = hwMap.get(Servo.class, "autoarm2");

        autoPush2           = hwMap.get(Servo.class, "autopush2");

        //this rgb is actually pretty big brain
        //look at revrobotics chart, get the number on the pattern you want, and multiply that number by 0.01
        rgb                 = hwMap.get(ServoImplEx.class, "rgb");
        PwmControl.PwmRange range = new PwmControl.PwmRange(995, 1995);
        rgb.setPwmRange(range);

    }
}