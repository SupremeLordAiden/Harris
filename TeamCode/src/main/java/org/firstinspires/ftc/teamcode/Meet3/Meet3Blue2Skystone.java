package org.firstinspires.ftc.teamcode.Meet3;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardWare1;
import org.firstinspires.ftc.teamcode.Movement1;

@Autonomous(name="Auto Yeet 3 Blue 2 Skystone", group="Linear Opmode")


public class Meet3Blue2Skystone extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).
    Movement1 movemento = new Movement1();
    HardWare1 robot1 = new HardWare1();


    private ColorSensor sensorColor;





    @Override
    public void runOpMode() {

        // you know that hardware file, here it is being called
        robot1.init(hardwareMap);
        movemento.init(hardwareMap, telemetry, this);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "color");


        robot1.foundationgrabber.setPosition(0.3);
        robot1.swipeServo.setPosition(0.925);
        robot1.grabbythingy.setPosition(0.8);
        robot1.armthingy.setPosition(0.76);
        robot1.capstonedropper.setPosition(0.5);


        robot1.autoPush.setPosition(0.6);
        robot1.autoArm.setPosition(0);

        robot1.autoPush2.setPosition(1);
        robot1.autoArm2.setPosition(1);

        robot1.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot1.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot1.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot1.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("oy hya ya");
        telemetry.update();



        //are you really that excited to start? calm calm down
        int SkystonePosition1 = 0;
        //0 is not found
        //1 is left side
        //2 is middle
        //3 is right

        int SkystonePossiblePosition = 1;

        boolean skystone = false;
        robot1.rgb.setPosition(0.92);
        waitForStart();

        movemento.forwardMMwithDistance(250, 0.5, 10);
        //movemento.straightMM(220, 0.5);
        sleep(1000);
        while (skystone == false) {
            double skystonevalue = (sensorColor.red() * sensorColor.green()) / (sensorColor.blue() * sensorColor.blue());
            if (skystonevalue < 2) {
                telemetry.addLine("skystone");
                skystone = true;
                SkystonePosition1 = SkystonePossiblePosition;
            } else if (skystonevalue >= 2) {
                telemetry.addLine("not skystone");

                if (SkystonePossiblePosition == 1) {
                    SkystonePossiblePosition = 2;
                    movemento.strafe(85, 0.5);
                }  else if (SkystonePossiblePosition == 2) {
                    movemento.strafe(30, 0.5);
                    SkystonePossiblePosition = 3;
                    SkystonePosition1 = 3;
                    skystone = true;
                }
            }
        }
        if (SkystonePosition1 == 1) {
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);
            robot1.autoArm.setPosition(0.46);
            movemento.strafe(-30, 0.5);
            movemento.straightMM(60, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-70, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.7);
            movemento.recalibrate(1, 90, 1);
            movemento.straightMM(150, 1);
            movemento.wallStrafe(2, -0.3);
            movemento.strafe(-10, 0.5);
            movemento.straightMM(150, 0.5);
            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);
            movemento.recalibrate(1, 90, 1);

            movemento.straightMM(-540, 0.5);
            movemento.strafe(-50, 0.5);
            movemento.rotate(-90, 0.7);
            movemento.recalibrate(3, 0, 1);


            movemento.straightMM(50, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-40, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.5);
            movemento.recalibrate(1, 90, 1);

            movemento.straightMM(470, 1);


            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);
            movemento.straightMM(-100, 0.5);
        } else if (SkystonePosition1 == 2) {
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);
            robot1.autoArm.setPosition(0.4);
            sleep(1000);
            //movemento.strafe(10, 0.5);
            movemento.straightMM(60, 0.3);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-70, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.7);
            movemento.recalibrate(1, 90, 1);
            movemento.strafe(50, 0.5);
            movemento.straightMM(400, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            movemento.recalibrate(3, 90, 1);

            movemento.straightMM(-630, 0.5);
            movemento.strafe(-50, 0.5);
            movemento.rotate(-90, 0.7);
            movemento.recalibrate(1, 0, 1);

            movemento.straightMM(60, 0.3);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-40, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.75);
            movemento.recalibrate(1, 90, 1);

            movemento.straightMM(580, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);
            movemento.straightMM(-100, 0.5);
        } else if (SkystonePosition1 == 3) {
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);
            movemento.strafe(7, 0.4);
            robot1.autoArm.setPosition(0.4);
            sleep(500);
            movemento.straightMM(60, 0.3);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-70, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.7);
            movemento.recalibrate(1, 90, 1);

            movemento.straightMM(400, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            movemento.recalibrate(1, 90, 1);

            movemento.straightMM(-580, 0.7);
            movemento.strafe(-50, 0.5);
            movemento.rotate(-90, 0.7);
            movemento.strafe(50, 0.5);

            movemento.straightMM(90, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-80, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.strafe(-50, 0.5);
            movemento.rotate(90, 0.5);
            movemento.recalibrate(1, 90, 1);
            movemento.strafe(50, 0.5);
            movemento.straightMM(630, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);
            movemento.straightMM(-120, 1);


        }

    }



}