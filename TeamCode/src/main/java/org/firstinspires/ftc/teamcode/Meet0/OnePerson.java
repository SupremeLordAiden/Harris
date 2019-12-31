package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class OnePerson extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).

    HardWare1 robot1 = new HardWare1();
    private ElapsedTime runtime  = new ElapsedTime();

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
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)

        waitForStart();




        //reset start time for how long the robot has run
        runtime.reset();

        //controls how fast the robot moves.
        double motorSpeed = 1;

        //speed of squishy speed naturally starts at 0
        double squishySpeed = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //change the speed of how fast the robot moves with the bumpers on Gamepad 1
            if (gamepad1.left_bumper == true) {
                if (motorSpeed == 0.25 ) {
                    motorSpeed = 0.25;
                } else {
                    motorSpeed = motorSpeed - 0.25;
                }
            } else if (gamepad1.right_bumper == true) {
                if (motorSpeed == 1) {
                    motorSpeed = 1;
                } else {
                    motorSpeed = motorSpeed + 0.25;
                }

            }
            telemetry.addData("Motor Speed:", motorSpeed);
            /*if (gamepad1.b == true) {
                squishySpeed = 0.5;
            } else if (gamepad1.y == true || gamepad1.a == true) {
                squishySpeed  = 0;
            } else if (gamepad1.x == true) {
                squishySpeed = -0.5;
            }
            */
            if (gamepad1.a) {
                squishySpeed = 0.5;
            } else if (gamepad1.y) {
                squishySpeed = -0.5;
            } else {
                squishySpeed = 0;
            }

            robot1.Squishy1.setPower(squishySpeed);
            robot1.Squishy2.setPower(-squishySpeed);
            //old code that can be used just in case
            /*double leftPower;
            double rightPower;

            //tells which stick does what
            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;

            //uses mathematics to figure out which direction the robot is going
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            //Adjusts the speed of  drive train
            robot1.leftDrive.setPower(-leftPower * motorSpeed);
            robot1.rightDrive.setPower(-rightPower * motorSpeed);
            */
            //this control allows for the left joystick to control direction fully positionally

            // yay math! hypotenus measures the length/ magnitude on the movement
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            //this part is basically multiplying the speed by the adjusted speed with the control speed set by the 2 buttons
            final double v1 = (r * Math.cos(robotAngle) + rightX) * motorSpeed;
            final double v2 = (r * Math.sin(robotAngle) - rightX) * motorSpeed;
            final double v3 = (r * Math.sin(robotAngle) + rightX) * motorSpeed;
            final double v4 = (r * Math.cos(robotAngle) - rightX) * motorSpeed;
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

            double servoPositionRacPinion = -gamepad2.left_stick_y;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
