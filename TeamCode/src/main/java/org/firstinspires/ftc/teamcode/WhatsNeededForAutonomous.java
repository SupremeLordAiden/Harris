package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@Autonomous(name="AutoEZ", group="Linear Opmode")
public class WhatsNeededForAutonomous extends LinearOpMode {
    
    // Declare OpMode members(motors, servos, and sensors).
    HardWare1 robot1 = new HardWare1();
    BNO055IMU imu;
    private ElapsedTime runtime     = new ElapsedTime();
    Orientation         lastAngles  = new Orientation();
    double correction, globalAngle;

    @Override
    public void runOpMode() {
        
        // you know that hardware file, here it is being called
        robot1.init(hardwareMap);

        //just some normal gyro stuff perfectly normal calm down i totally understand this and am not going crazy its 3 am haha im ok trust me this is just a typical comment :)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //time to init the imu
        imu.initialize(parameters);
        robot1.rightServo.setPosition(0.3);
        robot1.leftServo.setPosition(0.5);
        telemetry.addLine("Oh ya ya");
        telemetry.update();

        //are you really that excited to start? calm calm down


        waitForStart();

        //straightMM(100,0.5);
        //moveStraight(2, 0.3);
        //rotate(15,0.5);
        straightMM(50, 0.5);

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
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //meaning if you are going backwards
        if (encodercounts < 0) {

            double closeenough = encodercounts * 4/5;

            while (robot1.encoderMotor.getCurrentPosition() > closeenough) {
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

            while (robot1.encoderMotor.getCurrentPosition() > encodercounts && closeenough > robot1.leftDrive.getCurrentPosition()) {
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

            while (robot1.encoderMotor.getCurrentPosition() < closeenough) {
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
                telemetry.addData("5 harris Encoder Position", robot1.encoderMotor.getCurrentPosition());
                telemetry.addData("6 power:", power);
                telemetry.addData("7 test:", test);
                telemetry.update();

            }

            while (robot1.encoderMotor.getCurrentPosition() < encodercounts && closeenough < robot1.encoderMotor.getCurrentPosition()) {
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
                telemetry.addData("5 Encoder Position", robot1.encoderMotor.getCurrentPosition());
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
        robot1.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //meaning if you are going backwards
        if (encodercounts < 0) {

            double closeenough = encodercounts * 4/5;

            while (robot1.encoderMotor.getCurrentPosition() > closeenough) {
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

            while (robot1.encoderMotor.getCurrentPosition() > encodercounts && closeenough > robot1.encoderMotor.getCurrentPosition()) {
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

            while (robot1.encoderMotor.getCurrentPosition() < closeenough) {
                correction = checkDirection();
                double actualPower = correction / 5;
                robot1.leftDrive.setPower((power + actualPower));
                robot1.rightDrive.setPower(-(power - actualPower));
                robot1.backLeftDrive.setPower(-(power + actualPower));
                robot1.backRightDrive.setPower((power - actualPower));

                //tell the person some info
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 actual correction", actualPower);
                telemetry.addData("5 Encoder Position", robot1.encoderMotor.getCurrentPosition());
                telemetry.update();

            }

            while (robot1.encoderMotor.getCurrentPosition() < encodercounts && closeenough < robot1.encoderMotor.getCurrentPosition()) {
                correction = checkDirection();

                //this makes the correction power less strong so everything including imu/gyro can actually react
                double actualPower = correction / 5;

                //since we are really close, speed is needed to be slowed down
                double reductionspeed = power / 2;

                robot1.leftDrive.setPower(-(reductionspeed + actualPower));
                robot1.rightDrive.setPower(-(reductionspeed - actualPower));
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
