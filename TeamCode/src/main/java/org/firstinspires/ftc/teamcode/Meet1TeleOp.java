package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Meet1TeleOp", group="Linear Opmode")

public class Meet1TeleOp extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).

    HardWare1 robot1 = new HardWare1();
    private ElapsedTime runtime  = new ElapsedTime();

    private DistanceSensor distance;
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
        distance = hardwareMap.get(DistanceSensor.class, "distance");
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
        waitForStart();




        //reset start time for how long the robot has run
        runtime.reset();

        //controls how fast the robot moves.
        double motorSpeed = 1;

        //speed of squishy speed naturally starts at 0
        double squishySpeed = 0;

        double LineraSpeed = 0;
        boolean LineraWasActive = false;
        // run until the end of the match (driver presses STOP)
        int oldPosition = 0;
        int PositionofLinearSlide = 0;
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

            /*if (gamepad1.b == true) {
                squishySpeed = 0.5;
            } else if (gamepad1.y == true || gamepad1.a == true) {
                squishySpeed  = 0;
            } else if (gamepad1.x == true) {
                squishySpeed = -0.5;
            }
            */

            squishySpeed = gamepad2.left_trigger - gamepad2.right_trigger;

            robot1.Squishy1.setPower(squishySpeed);
            robot1.Squishy2.setPower(-squishySpeed);

            if (gamepad2.dpad_left == true) {
                robot1.grabbythingy.setPosition(0.80);
            } else if (gamepad2.dpad_right == true) {
                robot1.grabbythingy.setPosition(0.15);

            }

            if (gamepad2.dpad_down) {
                robot1.armthingy.setPosition(0.63);
            } else if (gamepad2.dpad_up) {
                robot1.armthingy.setPosition(0.75);
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






            // Show the elapsed game time and wheel power.

            if (robot1.TouchSense.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");

            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                robot1.LineraSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot1.LineraSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

            }
            if (gamepad2.y == true) {
                if (robot1.LineraSlide.getCurrentPosition() < 6000) {
                    LineraSpeed = 1;
                } else {
                    LineraSpeed = 0;
                }

            } else if (gamepad2.a == true) {
                if (robot1.TouchSense.getState() == true) {
                    if (robot1.LineraSlide.getCurrentPosition() > 1500) {
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
            robot1.capstonedropper.setPosition(gamepad2.right_stick_y);


            robot1.LineraSlide.setPower(LineraSpeed);

            if (gamepad1.left_stick_button == true) {
                robot1.foundationgrabber.setPosition(0.95);
            } else if (gamepad1.right_stick_button == true) {
                robot1.foundationgrabber.setPosition(0.3);
            }
            if (gamepad2.left_stick_y > 0.525) {
                robot1.swipeServo.setPosition(0.525);
            } else if (gamepad2.left_stick_y < 0) {
                robot1.swipeServo.setPosition(0.925);
            } else {
                robot1.swipeServo.setPosition(0.925 - gamepad2.left_stick_y);
            }




            if (gamepad2.left_bumper == true) {
                robot1.tapeMeasure.setPower(0.8);
            } else if (gamepad2.right_bumper == true) {
                robot1.tapeMeasure.setPower(-0.8);
            } else {
                robot1.tapeMeasure.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();

        }
    }
}
