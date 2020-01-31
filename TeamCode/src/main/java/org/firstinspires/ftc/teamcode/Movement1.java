package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

public class Movement1 {
    //hardware file includes all data about the motors, servos, etc
    HardWare1 robot1 = new HardWare1();

    //purpose is to route the
    LinearOpMode useless;
    //IMU is used to detect angle measures
    //in our case, we have 2 imu's
    //imuGlobal is to keep track of the global position, which restarts from the running of each program
    BNO055IMU imuGlobal;
    //imu is used to turn, but resets every time the function rotate() is run
    BNO055IMU imu;

    //ElapsedTime is to tell the drivers how much time there
    private ElapsedTime runtime     = new ElapsedTime();

    //lastAngles is to find what the last Angles was since the imu only goes through -179 to 180 degrees
    Orientation         lastAngles  = new Orientation();

    //correction from angle and the global angle
    double correction, globalAngle;

    //since hardwaremap is a function that can only be used in an autonomous or teleop code, we had to do a loop and this will equal hardwareMap in the init
    HardwareMap hwMap = null;
    //same thing for telemetry
    Telemetry telemetry     = null;

    /* Constructor */
    public Movement1() {

    }


    //when running a code, this init function makes it so that everything is initialized
    public void init(HardwareMap bhwMap, Telemetry btelemetry, LinearOpMode opMode) {

        //the hardwaremap thing only is in a linearopmode (autonomous or teleop), so hwMap and telemetry are now equal
        hwMap = bhwMap;
        telemetry = btelemetry;
        useless = opMode;
        //the parameters tells how to customize the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //this is how hardwareMpa pairs up the software object and the hardware object with the device
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuGlobal = hwMap.get(BNO055IMU.class, "imuGlobal");
        //time to init the imu
        imu.initialize(parameters);
        imuGlobal.initialize(parameters);

        //initializes the robot hardware1 file
        robot1.init(bhwMap);

        //sets all the motors in the robot to brake so the movements are more precise
        //left drive is the front left drive
        robot1.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot1.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot1.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot1.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    //recalibrate uses the Global imu to keep track of the angle relative to the starting position
    public void recalibrate(int seconds, int angle, double maxSpeed) {
        ElapsedTime currentTime  = new ElapsedTime();
        currentTime.reset();
        robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power;
        while ((currentTime.seconds() < seconds) && !(getGlobalAngle() == angle) && (useless.opModeIsActive())) {
            double porportionalseconds = (seconds - currentTime.seconds())/(seconds);
            if ((seconds/2) > currentTime.seconds()) {
                porportionalseconds = 0.5;
            }
            if (((Math.abs(angle - getGlobalAngle()) / 90) * maxSpeed) > 0.14) {
                power = (porportionalseconds) * (Math.abs(angle - getGlobalAngle()) / 30) * maxSpeed;
            } else {
                power = 0.14;
            }
            if (getGlobalAngle() > angle) {

                robot1.leftDrive.setPower(-power);
                robot1.rightDrive.setPower(power);
                robot1.backLeftDrive.setPower(-power);
                robot1.backRightDrive.setPower(power);
                telemetry.addData("Angle", getGlobalAngle());
                telemetry.update();
            } else if (getGlobalAngle() < angle) {
                robot1.leftDrive.setPower(power);
                robot1.rightDrive.setPower(-power);
                robot1.backLeftDrive.setPower(power);
                robot1.backRightDrive.setPower(-power);
                telemetry.addData("Angle", getGlobalAngle());
                telemetry.update();
            }
        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }

    //the forward distance goes 3/4 of integer MM, then continues moving forward until distance CM with half the distance of power
    public void forwardMMwithDistance(int MM, double power, int CM) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        if (encodercounts > 0) {
            //meaning if you are going forward
            double closeenough = encodercounts * 3/4;

            while ((-robot1.encoderMotor.getCurrentPosition() < closeenough) && (useless.opModeIsActive())) {
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                correction = checkDirection();
                double actualPower = correction;
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot1.leftDrive.setPower(-(power - actualPower));

                robot1.rightDrive.setPower(-(power + actualPower));
                robot1.backLeftDrive.setPower(-(power - actualPower));
                robot1.backRightDrive.setPower(-(power + actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 harris Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.addData("6 power:", power);
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();

            }

            while ((-robot1.encoderMotor.getCurrentPosition() < encodercounts) && (!(robot1.sensorDistance.getDistance(DistanceUnit.CM) < CM)) && (useless.opModeIsActive())) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 3;
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot1.leftDrive.setPower(-(reductionspeed - actualPower));
                robot1.rightDrive.setPower(-(reductionspeed + actualPower));
                robot1.backLeftDrive.setPower(-(reductionspeed - actualPower));
                robot1.backRightDrive.setPower(-(reductionspeed + actualPower));

                telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", robot1.sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.update();
            }

        }

        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);


    }

      /*
        this function is for going straight. using MM to measure how far you need to go, and just enter it.
        The power doesn't need to be slow because there is an auto slowdown when close
        The encoder is only on the leftDrive wheel,because the control hub can't really handle multiple i think
        */
    public void straightMM(int MM, double power) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //meaning if you are going backwards
        if (encodercounts < 0) {

            double closeenough = encodercounts * 4/5;

            while ((-robot1.encoderMotor.getCurrentPosition() > closeenough) && (useless.opModeIsActive())) {
                correction = checkDirection();
                double actualPower = correction * 3/4;
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot1.leftDrive.setPower((power + actualPower));
                robot1.rightDrive.setPower((power - actualPower));
                robot1.backLeftDrive.setPower((power + actualPower));
                robot1.backRightDrive.setPower((power - actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", robot1.leftDrive.getCurrentPosition());
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();
            }

            while ((-robot1.encoderMotor.getCurrentPosition() > encodercounts) && (closeenough > -robot1.leftDrive.getCurrentPosition()) && (useless.opModeIsActive())) {
                correction = checkDirection();
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction * 3/4;

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
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();
            }

        } else if (encodercounts > 0) {
            //meaning if you are going forward
            double closeenough = encodercounts * 4/5;

            while ((-robot1.encoderMotor.getCurrentPosition() < closeenough) && (useless.opModeIsActive())) {
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                correction = checkDirection();
                double actualPower = correction;
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot1.leftDrive.setPower(-(power - actualPower));

                robot1.rightDrive.setPower(-(power + actualPower));
                robot1.backLeftDrive.setPower(-(power - actualPower));
                robot1.backRightDrive.setPower(-(power + actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 harris Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.addData("6 power:", power);
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();

            }

            while ((-robot1.encoderMotor.getCurrentPosition() < encodercounts) && (closeenough < -robot1.encoderMotor.getCurrentPosition()) && (useless.opModeIsActive())) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot1.leftDrive.setPower(-(reductionspeed - actualPower));
                robot1.rightDrive.setPower(-(reductionspeed + actualPower));
                robot1.backLeftDrive.setPower(-(reductionspeed - actualPower));
                robot1.backRightDrive.setPower(-(reductionspeed + actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.update();
            }

        }

        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }


    /*
    wall straight runs the motors with time, with no feedback to keep in straight
    only used for ramming into walls
     */
    public void wallStraight(int time, double power) {
        runtime.reset();


        if (power > 0) {



            while ((runtime.seconds() < time) && (useless.opModeIsActive())) {

                robot1.leftDrive.setPower((power));
                robot1.rightDrive.setPower((power));
                robot1.backLeftDrive.setPower((power));
                robot1.backRightDrive.setPower((power));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);

                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();
            }



        } else if (power < 0) {



            while ((runtime.seconds() < time) && (useless.opModeIsActive())) {
                double actualPower = correction / 2;
                robot1.leftDrive.setPower(-(power));
                robot1.rightDrive.setPower(-(power));
                robot1.backLeftDrive.setPower(-(power));
                robot1.backRightDrive.setPower(-(power));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.update();
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }



        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }





    /*
    this function is for strafing, and using encoder. The encoder is only on the leftDrive wheel. Just measure the d
     */
    public void strafe(int MM, double power) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("data:", robot1.encoderMotor.getCurrentPosition());
        telemetry.update();
        //meaning if you are going backwards
        if (encodercounts > 0) {

            double closeenough = encodercounts * 4/5;

            while ((-robot1.encoderMotor.getCurrentPosition() < closeenough) && (useless.opModeIsActive())) {
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                correction = checkDirection();
                double actualPower = correction / 3;
                robot1.leftDrive.setPower(-(power - actualPower));
                robot1.rightDrive.setPower((power- actualPower));
                robot1.backLeftDrive.setPower((power + actualPower));
                robot1.backRightDrive.setPower(-(power + actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);

                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();
            }

            while ((-robot1.encoderMotor.getCurrentPosition() < encodercounts) && (closeenough < -robot1.encoderMotor.getCurrentPosition()) && (useless.opModeIsActive())) {
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 2;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;

                robot1.leftDrive.setPower(-(reductionspeed - actualPower));
                robot1.rightDrive.setPower((reductionspeed - actualPower));
                robot1.backLeftDrive.setPower((reductionspeed + actualPower));
                robot1.backRightDrive.setPower(-(reductionspeed + actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.update();
            }

        } else if (encodercounts < 0) {
            //meaning if you are going forward
            double closeenough = encodercounts * 4/5;

            while ((-robot1.encoderMotor.getCurrentPosition() > closeenough) && (useless.opModeIsActive())) {
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                correction = checkDirection();
                double actualPower = correction / 2;
                robot1.leftDrive.setPower((power + actualPower));
                robot1.rightDrive.setPower(-(power + actualPower));
                robot1.backLeftDrive.setPower(-(power - actualPower));
                robot1.backRightDrive.setPower((power - actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.update();
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            while ((-robot1.encoderMotor.getCurrentPosition() < encodercounts) && (closeenough < robot1.encoderMotor.getCurrentPosition()) && (useless.opModeIsActive())) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 2;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 3;
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot1.leftDrive.setPower((reductionspeed + actualPower));
                robot1.rightDrive.setPower(-(reductionspeed + actualPower));
                robot1.backLeftDrive.setPower(-(reductionspeed - actualPower));
                robot1.backRightDrive.setPower((reductionspeed - actualPower));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);

                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());
                telemetry.update();
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }

    /*
      wall strafe runs the motors with time, with no feedback to keep in straight
    only used for ramming into walls
     */

    public void wallStrafe(int time, double power) {
        runtime.reset();



        //setting the power negative strafes right

            while ((runtime.seconds() < time) && (useless.opModeIsActive())) {

                robot1.leftDrive.setPower((power));
                robot1.rightDrive.setPower(-(power));
                robot1.backLeftDrive.setPower(-(power));
                robot1.backRightDrive.setPower((power));


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);

                telemetry.addData("5 Encoder Position", -robot1.encoderMotor.getCurrentPosition());

                telemetry.update();

        }
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);

    }

    /*
    rotate restarts the imu and rotates to the certain degree, but if the degree is passed, there is no turning back or feedback loop
     */
    public void rotate(int degrees, double power) {
        resetAngle();
        robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (degrees < 0)
        {   // turn left.

            robot1.leftDrive.setPower(-power);
            robot1.rightDrive.setPower(power);
            robot1.backLeftDrive.setPower(-power);
            robot1.backRightDrive.setPower(power);
        } else if (degrees > 0)
        {   // turn right.
            robot1.leftDrive.setPower(power);
            robot1.rightDrive.setPower(-power);
            robot1.backLeftDrive.setPower(power);
            robot1.backRightDrive.setPower(-power);
        }

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while ((getAngle() == 0) && (useless.opModeIsActive())) {
                telemetry.addData("turning", getAngle());
                telemetry.update();
            }
            double howcloseshouldIbe = degrees * 4 / 5;
            while ((getAngle() > howcloseshouldIbe) && (useless.opModeIsActive()))  {
                telemetry.addData("turning right", getAngle());
                telemetry.update();
            }
            while((getAngle() < howcloseshouldIbe) && (degrees < getAngle()) && (useless.opModeIsActive())) {

                double powerwhenclose = power/2;

                robot1.leftDrive.setPower(-powerwhenclose);
                robot1.rightDrive.setPower(powerwhenclose);
                robot1.backLeftDrive.setPower(-powerwhenclose);
                robot1.backRightDrive.setPower(powerwhenclose);

                telemetry.addData("turning right", getAngle());
                telemetry.update();
            }
        } else {    // left turn.
            double howcloseshouldIbe = degrees * 4 / 5;

            while ((getAngle() < howcloseshouldIbe) && (useless.opModeIsActive())) {
                telemetry.addData("turning left", getAngle());
                telemetry.update();
            }
            while((getAngle() < degrees) && (howcloseshouldIbe < getAngle()) && (useless.opModeIsActive())) {

                double powerwhenclose = power/2;

                robot1.leftDrive.setPower(powerwhenclose);
                robot1.rightDrive.setPower(-powerwhenclose);
                robot1.backLeftDrive.setPower(powerwhenclose);
                robot1.backRightDrive.setPower(-powerwhenclose);
                telemetry.addData("turning left", getAngle());
                telemetry.update();
            }
        }


        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        robot1.backLeftDrive.setPower(0);
        robot1.backRightDrive.setPower(0);
    }


    /*
   checkdirection makes sure the number is positive
     */
    public double checkDirection()
    {

        double correction, angle, gain = .1;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    /*
    checkdirection is used to find the direction, and the imu has a property of having a range from -179 ti 180 degrees
    this function allows for it to tally up to 360 degrees
     */
    public double getAngle()
    {


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

    /*
    this is the global angle and to get the global angle and position
     */
    public double getGlobalAngle()
    {


        Orientation angles = imuGlobal.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        return angles.firstAngle;
    }
    /*
    resets the imu angle for the rotate function
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }



}