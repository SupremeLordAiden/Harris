package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="ILTTeleOp", group="Linear Opmode")

public class ILTTeleOp extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).

    HardWare1 robot1 = new HardWare1();
    private ElapsedTime runtime  = new ElapsedTime();

    double threadedDistance         = 0;
    double rightThreadedDistance    = 0;
    double leftThreadedDistance     = 0;


    @Override
    public void runOpMode() {
        //Tell driver the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot1.init(hardwareMap);

        robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot1.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot1.backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot1.backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        robot1.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot1.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot1.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot1.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)

        robot1.autoPush.setPosition(0.8);
        robot1.autoArm.setPosition(0);

        robot1.rgb.setPosition(0.79);

        Thread sensorThread = new sensorThread();
        waitForStart();
        robot1.autoPush2.setPosition(1);
        robot1.autoArm2.setPosition(1);




        //reset start time for how long the robot has run
        runtime.reset();

        //controls how fast the robot moves.
        double motorSpeed = 1;

        //speed of squishy speed naturally starts at 0
        double squishySpeed = 0;

        double LineraSpeed = 0;

        sensorThread.start();
        boolean isChainOutside = true;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot1.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot1.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot1.backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot1.backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot1.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot1.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot1.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot1.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //change the speed of how fast the robot moves with the bumpers on Gamepad 1
            if (gamepad1.left_bumper == true) {
                motorSpeed = 0.4;
            } else if (gamepad1.right_bumper == true) {
                motorSpeed  = 1;
            }



            squishySpeed = gamepad2.left_trigger - gamepad2.right_trigger;

            robot1.Squishy1.setPower(squishySpeed);
            robot1.Squishy2.setPower(-squishySpeed);


            if (gamepad2.dpad_left) {
                robot1.grabbythingy.setPosition(0.80);
                isChainOutside = false;
            } else if (gamepad2.dpad_right) {

                robot1.grabbythingy.setPosition(0.15);
                isChainOutside = true;

            } else if (gamepad2.left_stick_button) {
                robot1.grabbythingy.setPosition(0.05);
                isChainOutside = true;
            }

            if (gamepad2.dpad_down) {
                robot1.armthingy.setPosition(0.63);
            } else if (gamepad2.dpad_up) {
                if ((threadedDistance > 5) || (isChainOutside)) {
                    robot1.armthingy.setPosition(0.75);
                }
            }





            //this control allows for the left joystick to control direction fully positionally

            // yay math! hypotenuse measures the length/ magnitude on the movement
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            //this part is basically multiplying the speed by the adjusted speed with the control speed set by the 2 buttons
            final double v1 = (r * Math.sin(robotAngle) - rightX) * motorSpeed;
            final double v2 = (r * Math.cos(robotAngle) + rightX) * motorSpeed;
            final double v3 = (r * Math.cos(robotAngle) - rightX) * motorSpeed;
            final double v4 = (r * Math.sin(robotAngle) + rightX) * motorSpeed;
            //in hardware, need to set names called backLeftDrive and backRightDrive
            robot1.leftDrive.setPower(v1);
            robot1.rightDrive.setPower(v2);
            robot1.backLeftDrive.setPower(v3);
            robot1.backRightDrive.setPower(v4);

            //time to show this bueatiful data to the viewer, very nessecary
            telemetry.addData("Front Left Drive", v1);
            telemetry.addData("Front Right Drive", v2);
            telemetry.addData("Back Left Drive", v3);
            telemetry.addData("Back Right Drive", v4);







            /*touch sensor */
            if (robot1.TouchSense.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");

            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                robot1.LineraSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot1.LineraSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

            }




            /*Linear Slide Speed Control*/
            if (gamepad2.y == true) {
                if (robot1.LineraSlide.getCurrentPosition() < 6000) {
                    LineraSpeed = 1;
                } else {
                    LineraSpeed = 0;
                }

            } else if (gamepad2.a == true) {
                if (robot1.TouchSense.getState() == true) {
                    if (robot1.LineraSlide.getCurrentPosition() > 500) {
                        LineraSpeed = -1;
                    } else {
                        LineraSpeed = -0.25;
                    }
                } else {
                    LineraSpeed = 0;
                }
            } else {
                LineraSpeed = 0;
            }
            robot1.LineraSlide.setPower(LineraSpeed);




            /*capstone dropper*/
            robot1.capstonedropper.setPosition(gamepad2.right_stick_y);




            if (gamepad1.left_stick_button == true) {
                robot1.foundationgrabber.setPosition(0.95);
            } else if (gamepad1.right_stick_button == true) {
                robot1.foundationgrabber.setPosition(0.3);
            }
            if (threadedDistance > 5) {
                if (gamepad2.left_stick_y > 0.525) {
                    robot1.swipeServo.setPosition(0.525);
                } else if (gamepad2.left_stick_y < 0) {
                    robot1.swipeServo.setPosition(0.925);
                } else {
                    robot1.swipeServo.setPosition(0.925 - gamepad2.left_stick_y);
                }
            } else {
                robot1.swipeServo.setPosition(0.525);
            }





            if (gamepad2.left_bumper == true) {
                robot1.tapeMeasure.setPower(0.8);
            } else if (gamepad2.right_bumper == true) {
                robot1.tapeMeasure.setPower(-0.8);
            } else {
                robot1.tapeMeasure.setPower(0);
            }

            if (threadedDistance < 5) {
                //rainbow
                robot1.rgb.setPosition(0.16);

            } else {
                if ((rightThreadedDistance < 818) || (leftThreadedDistance < 818)) {
                    //leds
                    if ((rightThreadedDistance < 50) && (rightThreadedDistance > 25) && (leftThreadedDistance < 50) && (leftThreadedDistance > 25)) {
                        //green
                        robot1.rgb.setPosition(0.89);
                    } else if ((rightThreadedDistance < 25) || (leftThreadedDistance < 25)) {
                        //red
                        robot1.rgb.setPosition(0.81);
                    } else if ((rightThreadedDistance > 50) || (leftThreadedDistance > 50)) {
                        //blue
                        robot1.rgb.setPosition(0.92);
                    }
                } else if ((rightThreadedDistance == 819) && (leftThreadedDistance == 819)) {
                    //green
                    robot1.rgb.setPosition(0.89);
                }
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("distance", threadedDistance);
            telemetry.addData("right Distance", rightThreadedDistance);
            telemetry.addData("left Distance", leftThreadedDistance);
            telemetry.addData("grabbythingy", robot1.grabbythingy.getPosition());
            telemetry.update();

        }
        sensorThread.interrupt();
    }
    private class sensorThread extends Thread {
        public sensorThread() {
                this.setName("sensorThread");
            }
            @Override
            public void run() {
                try {
                    while (!isInterrupted()) {
                        threadedDistance        = robot1.distance.getDistance(DistanceUnit.CM);
                        leftThreadedDistance    = robot1.distanceLeft.getDistance(DistanceUnit.CM);
                        rightThreadedDistance   = robot1. distanceRight.getDistance(DistanceUnit.CM);
                        idle();
                    }
                } //catch (InterruptedException e) {
                //dab
                // }
                catch (Exception e) {
                    //hi
                }
        }
    }
}

