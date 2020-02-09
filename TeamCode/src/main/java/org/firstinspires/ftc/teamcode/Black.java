package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Black", group="Linear Opmode")

public class Black extends LinearOpMode {

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
            robot1.rgb.setPosition(1);

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

