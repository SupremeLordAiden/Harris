package org.firstinspires.ftc.teamcode;
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

    HardWare1 robot1 = new HardWare1();

    BNO055IMU imuGlobal;
    BNO055IMU imu;
    private ElapsedTime runtime     = new ElapsedTime();
    Orientation         lastAngles  = new Orientation();
    double correction, globalAngle;
    HardwareMap hwMap = null;
    Telemetry telemetry     = null;

    /* Constructor */
    public Movement1() {

    }
         /*
    this function is for going straight. using MM to measure how far you need to go, and just enter it.
    The power doesn't need to be slow because there is an auto slowdown when close
    The encoder is only on the leftDrive wheel,because the control hub can't really handle multiple i think
     */



    public void init(HardwareMap bhwMap, Telemetry btelemetry) {

        hwMap = bhwMap;
        telemetry = btelemetry;

        //robot1.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        imu = hwMap.get(BNO055IMU.class, "imu");
        imuGlobal = hwMap.get(BNO055IMU.class, "imuGlobal");
        //time to init the imu
        imu.initialize(parameters);
        imuGlobal.initialize(parameters);
        robot1.init(bhwMap);
        robot1.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot1.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot1.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot1.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void recalibrate(int seconds, int angle, double maxSpeed) {
        ElapsedTime currentTime  = new ElapsedTime();
        currentTime.reset();
        robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power;
        while ((currentTime.seconds() < seconds) && !(getGlobalAngle() == angle)) {
            if (((Math.abs(angle - getGlobalAngle()) / 90) * maxSpeed) > 0.1) {
                power = ((seconds - currentTime.seconds())/(seconds)) * (Math.abs(angle - getGlobalAngle()) / 30) * maxSpeed;
            } else {
                power = 0.1;
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
    public void forwardMMwithDistance(int MM, double power, int CM) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        if (encodercounts > 0) {
            //meaning if you are going forward
            double closeenough = encodercounts * 3/4;

            while (-robot1.encoderMotor.getCurrentPosition() < closeenough) {
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

            while ((-robot1.encoderMotor.getCurrentPosition() < encodercounts) && (!(robot1.sensorDistance.getDistance(DistanceUnit.CM) < CM))) {
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

    public void straightMM(int MM, double power) {
        resetAngle();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //meaning if you are going backwards
        if (encodercounts < 0) {

            double closeenough = encodercounts * 4/5;

            while (-robot1.encoderMotor.getCurrentPosition() > closeenough) {
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

            while (-robot1.encoderMotor.getCurrentPosition() > encodercounts && closeenough > -robot1.leftDrive.getCurrentPosition()) {
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

            while (-robot1.encoderMotor.getCurrentPosition() < closeenough) {
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

            while (-robot1.encoderMotor.getCurrentPosition() < encodercounts && closeenough < -robot1.encoderMotor.getCurrentPosition()) {
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

    public int forwardMMwithIntake(int MM, double power) {
        resetAngle();
        ElapsedTime porportional  = new ElapsedTime();

        double encodercounts = 7.672 * MM;

        //since we are only going forward, I am only using one motor to measure the enocoder
        //and since there is straightness correction, it should work
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        if (encodercounts > 0) {

            robot1.Squishy1.setPower(-0.6);
            robot1.Squishy2.setPower(0.6);
            while ((-robot1.encoderMotor.getCurrentPosition() < encodercounts) && !(robot1.IntakeSensor.getDistance(DistanceUnit.CM) < 10)) {
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
                telemetry.addData("7 distance", robot1.IntakeSensor.getDistance(DistanceUnit.CM));
                robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.update();

            }
            porportional.reset();
            robot1.Squishy1.setPower(0);
            robot1.Squishy2.setPower(0);
            robot1.leftDrive.setPower(0);
            robot1.rightDrive.setPower(0);
            robot1.backLeftDrive.setPower(0);
            robot1.backRightDrive.setPower(0);
            if (robot1.IntakeSensor.getDistance(DistanceUnit.CM) < 10) {
                while (robot1.IntakeSensor.getDistance(DistanceUnit.CM) < 10) {
                    robot1.Squishy1.setPower(-0.8);
                    robot1.Squishy2.setPower(0.8);
                    if (porportional.seconds() < 1.2) {
                        double swipePosition = 1 - (porportional.seconds()/3) - 0.075;
                        robot1.swipeServo.setPosition(swipePosition);
                    } else {
                        robot1.swipeServo.setPosition(0.525);
                    }
                }
                robot1.swipeServo.setPosition(0.525);
            }


        }



        int returnMM = (int) (-robot1.encoderMotor.getCurrentPosition() / 7.672);
        return returnMM;

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

            while (-robot1.encoderMotor.getCurrentPosition() < closeenough) {
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

            while (-robot1.encoderMotor.getCurrentPosition() < encodercounts && closeenough < -robot1.encoderMotor.getCurrentPosition()) {
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

            while (-robot1.encoderMotor.getCurrentPosition() > closeenough) {
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

            while (-robot1.encoderMotor.getCurrentPosition() < encodercounts && closeenough < robot1.encoderMotor.getCurrentPosition()) {
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
            while (getAngle() == 0) {
                telemetry.addData("turning", getAngle());
                telemetry.update();
            }
            double howcloseshouldIbe = degrees * 4 / 5;
            while (getAngle() > howcloseshouldIbe) {
                telemetry.addData("turning right", getAngle());
                telemetry.update();
            }
            while(getAngle() < howcloseshouldIbe && degrees < getAngle()) {

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

            while (getAngle() < howcloseshouldIbe) {
                telemetry.addData("turning left", getAngle());
                telemetry.update();
            }
            while(getAngle() < degrees && howcloseshouldIbe < getAngle()) {

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




    public double checkDirection()
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
    public double getAngle()
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

    public double getGlobalAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imuGlobal.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        return angles.firstAngle;
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


}