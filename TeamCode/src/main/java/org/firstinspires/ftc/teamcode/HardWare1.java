package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardWare1 {
    /* Public OpMode members. */
    public DcMotor leftDrive        = null;
    public DcMotor rightDrive       = null;

    public DcMotor Squishy1         = null;
    public DcMotor Squishy2         = null;

    public DcMotor encoderMotor     = null;

    public DcMotor LineraSlide      = null;

    public DcMotor backLeftDrive    = null;
    public DcMotor backRightDrive   = null;



    public Servo swipeServo         = null;
    public Servo armthingy          = null;
    public Servo grabbythingy       = null;
    public Servo foundationgrabber  = null;




    public DigitalChannel TouchSense   = null;



    public static final double MID_SERVO        = 0.5;
    public static final double ARM_UP_POWER     = 0.45;
    public static final double ARM_DOWN_POWER   = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap          = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardWare1() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        /*
        these 4 are GoBilda 435 RPM motors with a ratio of 13.7:1
        There are 383.6 Encoder Countable Events  per Revolution
        that means that there will be 767.2 encoder counts per output shaft
        diameter of the wheel is 100 mm. So for every 767.2 encoder counts, there will be 100 mm of movement forward
        */

        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        backLeftDrive = hwMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");
        encoderMotor    = hwMap.get(DcMotor.class, "encoder");




        TouchSense = hwMap.get(DigitalChannel.class, "TouchSense");

        TouchSense.setMode(DigitalChannel.Mode.INPUT);
        /*
        the 2 intake wheels are GoBilda 223 RPM motors with a rattio of 26.9:1
        These are 188.3 Encoder Cycles Per Revolution
         */
        Squishy1 = hwMap.get(DcMotor.class, "squishy1");
        Squishy2 = hwMap.get(DcMotor.class, "squishy2");

        LineraSlide = hwMap.get(DcMotor.class, "LineraSlide");

        //might need to change
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // these motors are 435 rpm motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        LineraSlide.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.



        // Define and initialize ALL installed servos.
        //right now as of 10/21/19, there are no servos and so are being used as test

        armthingy           = hwMap.get(Servo.class, "armthingy");
        grabbythingy        = hwMap.get(Servo.class, "grabbythingy");

        swipeServo          = hwMap.get(Servo.class, "swipe");
        foundationgrabber   = hwMap.get(Servo.class, "foundationgrabber");
    }
}