package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Meet 0", group="Linear Opmode")

public class TestAuto2 extends LinearOpMode {
    //detection Stuff
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    int SkystonePosition1 = 0;
    //0 is not found
    //1 is left side
    //2 is middle
    //3 is right



    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName;

    // Declare OpMode members(motors, servos, and sensors).
    HardWare1 robot1 = new HardWare1();
    BNO055IMU imu;
    private ElapsedTime runtime     = new ElapsedTime();
    Orientation         lastAngles  = new Orientation();
    double correction, globalAngle;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = " AbgqVUj/////AAABmQJK3C3ywkbBs0xOk9ikb0kYt5NS2MeAfNeX/3J/zV8O2s7cr8Y79sdcDs2TBtBquC5T9bmThY0up87lwOqQfuZsq38AfzSRjJZ5qmhPx94e/bkewmh8RgKErgcE0yAbIWS1QwLZKzBFOA2stTpBUiOXBhkX+p07OaFO0sum959QYli9xdmmBZg/9GzusrqedKMcxXY/4+F+H8ui9B49EBrB+MjFZ6Vs796rp4aUKP5xhIqXY1vR6ylvhJHYJtyM/tGmMZB8tgcw2n+p2/S882jafm9PcbYWVYqGvhMswKB225pMZG+R5dotu2kyC7PThRYCOQ+GQ+FJBWTnqT6Nw9w+6FndzDfTRQygi99K2joN";

        parameters.cameraName = webcamName;

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        float StoneRecognitionx     = 0;
        float SkyStoneRecognitionx  = 0;
        float StonevsSkyStonex      = 0;
        if (tfod != null) {
            tfod.activate();
        }

        // you know that hardware file, here it is being called
        robot1.init(hardwareMap);

        //just some normal gyro stuff perfectly normal calm down i totally understand this and am not going crazy
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //time to init the imu
        imu.initialize(parameters);
        telemetry.addLine("Oh ya ya");
        telemetry.update();

        //are you really that excited to start?32   ` calm calm down


        waitForStart();




        moveStraight(2, 0.3);
        rotate(90,0.5);

        //strafe(4, 0.1);
        //straightMM(1000,0.5);
        /*

        rotate(90,0.5);
        moveStraight(5, 0.5);
        rotate(180,0.5);
        moveStraight(5, 0.5);
        rotate(-90,0.5);
        moveStraight(5, 0.5);
        rotate(180, 0.5);
        */

    }
    /*
    this function is for going straight. using MM to measure how far you need to go, and just enter it.
    The power doesn't need to be slow because there is an auto slowdown when close
    The encoder is only on the leftDrive wheel,because the control hub can't really handle multiple i think
     */

    private void straightMM(int MM, double power) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //meaning if you are going backwards
        if (encodercounts < 0) {

            double closeenough = encodercounts * 4/5;

            while (robot1.leftDrive.getCurrentPosition() > closeenough) {
                correction = checkDirection();
                double actualPower = correction / 5;
                robot1.leftDrive.setPower((power + actualPower));
                robot1.rightDrive.setPower((power - actualPower));
                robot1.backLeftDrive.setPower((power + actualPower));
                robot1.backRightDrive.setPower((power - actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", robot1.leftDrive.getCurrentPosition());
                telemetry.update();
            }

            while (robot1.leftDrive.getCurrentPosition() > encodercounts && closeenough > robot1.leftDrive.getCurrentPosition()) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 5;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;

                robot1.leftDrive.setPower((reductionspeed + actualPower));
                robot1.rightDrive.setPower((reductionspeed - actualPower));
                robot1.backLeftDrive.setPower((reductionspeed + actualPower));
                robot1.backRightDrive.setPower((reductionspeed - actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", robot1.leftDrive.getCurrentPosition());
                telemetry.update();
            }

        } else if (encodercounts > 0) {
            //meaning if you are going forward
            double closeenough = encodercounts * 4/5;

            while (robot1.leftDrive.getCurrentPosition() < closeenough) {
                correction = checkDirection();
                double actualPower = correction / 5;
                robot1.leftDrive.setPower(-(power + actualPower));
                //robot1.rightDrive.setPower(-(power - actualPower));
                double test = -(power - actualPower);
                //robot1.rightDrive.setPower(test);
                robot1.rightDrive.setPower(-(power - actualPower));
                robot1.backLeftDrive.setPower(-(power + actualPower));
                robot1.backRightDrive.setPower(-(power - actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 harris Encoder Position", robot1.leftDrive.getCurrentPosition());
                telemetry.addData("6 power:", power);
                telemetry.addData("7 test:", test);
                telemetry.update();

            }

            while (robot1.leftDrive.getCurrentPosition() < encodercounts && closeenough < robot1.leftDrive.getCurrentPosition()) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 5;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;

                robot1.leftDrive.setPower(-(reductionspeed + actualPower));
                robot1.rightDrive.setPower(-(reductionspeed - actualPower));
                robot1.backLeftDrive.setPower(-(reductionspeed + actualPower));
                robot1.backRightDrive.setPower(-(reductionspeed - actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", robot1.leftDrive.getCurrentPosition());
                telemetry.update();
            }

        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }






    /*
    this function is for going in a general direction. using time to measure how far you need to go, and just enter it.
    The power does need to be small cuz accuracy is slighty a problem
     */
    private void generalDirection(double leftDrive, double rightDrive, double backLeftDrive, double backrightDrive, double time) {
        resetAngle();
        runtime.reset();

        while (runtime.time() < time) {
            correction = checkDirection();
            double actualPower = correction/5;
            robot1.leftDrive.setPower(-(leftDrive + actualPower));
            robot1.rightDrive.setPower(-(rightDrive - actualPower));
            robot1.backLeftDrive.setPower(-(backLeftDrive + actualPower));
            robot1.backRightDrive.setPower(-(backrightDrive - actualPower));


            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);

            telemetry.addData("Time:", runtime.time());
            telemetry.update();
        }
    }


    /*
    this function is for strafing, and using encoder. The encoder is only on the leftDrive wheel. Just measure the d
     */
    private void strafe(int MM, double power) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //meaning if you are going backwards
        if (encodercounts < 0) {

            double closeenough = encodercounts * 4/5;

            while (robot1.leftDrive.getCurrentPosition() > closeenough) {
                correction = checkDirection();
                double actualPower = correction / 5;
                robot1.leftDrive.setPower((power + actualPower));
                robot1.rightDrive.setPower(-(power - actualPower));
                robot1.backLeftDrive.setPower((power + actualPower));
                robot1.backRightDrive.setPower(-(power - actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);

                telemetry.addData("4 actual correction", actualPower);
                telemetry.update();
            }

            while (robot1.leftDrive.getCurrentPosition() > encodercounts && closeenough > robot1.leftDrive.getCurrentPosition()) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 5;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;

                robot1.leftDrive.setPower((reductionspeed + actualPower));
                robot1.rightDrive.setPower(-(reductionspeed - actualPower));
                robot1.backLeftDrive.setPower((reductionspeed + actualPower));
                robot1.backRightDrive.setPower(-(reductionspeed - actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);

                telemetry.update();
            }

        } else if (encodercounts > 0) {
            //meaning if you are going forward
            double closeenough = encodercounts * 4/5;

            while (robot1.leftDrive.getCurrentPosition() < closeenough) {
                correction = checkDirection();
                double actualPower = correction / 5;
                robot1.leftDrive.setPower(-(power + actualPower));
                robot1.rightDrive.setPower((power - actualPower));
                robot1.backLeftDrive.setPower(-(power + actualPower));
                robot1.backRightDrive.setPower((power - actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);

                telemetry.update();

            }

            while (robot1.leftDrive.getCurrentPosition() < encodercounts && closeenough < robot1.leftDrive.getCurrentPosition()) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 5;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;

                robot1.leftDrive.setPower(-(reductionspeed + actualPower));
                robot1.rightDrive.setPower((reductionspeed - actualPower));
                robot1.backLeftDrive.setPower(-(reductionspeed + actualPower));
                robot1.backRightDrive.setPower((reductionspeed - actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);

                telemetry.addData("4 actual correction", actualPower);
                telemetry.update();
            }

        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }

    private void rotate(int degrees, double power) {
        resetAngle();

        if (degrees < 0)
        {   // turn left.
            robot1.leftDrive.setPower(power);
            robot1.rightDrive.setPower(-power);
            robot1.backLeftDrive.setPower(power);
            robot1.backRightDrive.setPower(-power);
        } else if (degrees > 0)
        {   // turn right.
            robot1.leftDrive.setPower(-power);
            robot1.rightDrive.setPower(power);
            robot1.backLeftDrive.setPower(-power);
            robot1.backRightDrive.setPower(power);
        }

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("turning", getAngle());
                telemetry.update();
            }
            double howcloseshouldIbe = degrees * 4 / 5;
            while (opModeIsActive() && getAngle() > howcloseshouldIbe) {
                telemetry.addData("turning right", getAngle());
                telemetry.update();
            }
            while(opModeIsActive() && getAngle() < howcloseshouldIbe && degrees < getAngle()) {

                double powerwhenclose = power/2;

                robot1.leftDrive.setPower(powerwhenclose);
                robot1.rightDrive.setPower(-powerwhenclose);
                robot1.backLeftDrive.setPower(powerwhenclose);
                robot1.backRightDrive.setPower(-powerwhenclose);

                telemetry.addData("turning right", getAngle());
                telemetry.update();
            }
        } else {    // left turn.
            double howcloseshouldIbe = degrees * 4 / 5;

            while (opModeIsActive() && getAngle() < howcloseshouldIbe) {
                telemetry.addData("turning left", getAngle());
                telemetry.update();
                while(opModeIsActive() &&  getAngle() < degrees && howcloseshouldIbe < getAngle()) {

                    double powerwhenclose = power/2;

                    robot1.leftDrive.setPower(-powerwhenclose);
                    robot1.rightDrive.setPower(powerwhenclose);
                    robot1.backLeftDrive.setPower(-powerwhenclose);
                    robot1.backRightDrive.setPower(powerwhenclose);
                    telemetry.addData("turning left", getAngle());
                    telemetry.update();
                }
            }

        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);
    }

    private void moveStraight(double time, double power) {

        resetAngle();
        runtime.reset();

        while (runtime.time() < time) {
            correction = checkDirection();
            double actualPower = correction/5;
            robot1.leftDrive.setPower(-(power + actualPower));
            robot1.rightDrive.setPower(-(power - actualPower));
            robot1.backLeftDrive.setPower(-(power + actualPower));
            robot1.backRightDrive.setPower(-(power - actualPower));


            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);

            telemetry.addData("Time:", runtime.time());
            telemetry.update();
        }
    }
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


}