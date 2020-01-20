package org.firstinspires.ftc.teamcode.Meet3;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardWare1;
import org.firstinspires.ftc.teamcode.Movement1;

@Autonomous(name="Auto Yeet 3 Blue Skystone", group="Linear Opmode")


public class Meet3BlueSkystone extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).
    Movement1 movemento = new Movement1();
    HardWare1 robot1 = new HardWare1();


    private ColorSensor sensorColor;





    @Override
    public void runOpMode() {

        // you know that hardware file, here it is being called
        robot1.init(hardwareMap);
        movemento.init(hardwareMap, telemetry);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "color");


        robot1.foundationgrabber.setPosition(0.3);
        robot1.swipeServo.setPosition(0.925);
        robot1.grabbythingy.setPosition(0.8);
        robot1.armthingy.setPosition(0.75);
        robot1.capstonedropper.setPosition(0.5);


        robot1.autoPush.setPosition(0.6);
        robot1.autoArm.setPosition(0);

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
                movemento.strafe(85, 0.5);
                if (SkystonePossiblePosition == 1) {
                    SkystonePossiblePosition = 2;
                }  else if (SkystonePossiblePosition == 2) {
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
            movemento.recalibrate(3, 90, 1);
            movemento.straightMM(500, 1);
            movemento.rotate(-80, 0.5);
            movemento.recalibrate(1, 10, 1);
            movemento.straightMM(90, 0.5);
            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.straightMM(-50, 0.5);
            movemento.rotate(-170, 0.5);
            movemento.straightMM(-100, 0.5);
            robot1.foundationgrabber.setPosition(0.975);
            sleep(1500);
            movemento.straightMM(230, 0.5);
            movemento.rotate(80, 0.5);
            robot1.foundationgrabber.setPosition(0.3);
            sleep(1000);
            movemento.straightMM(-50, 0.5);

            movemento.strafe(-70, 1);

            movemento.straightMM(300, 1);

        } else if (SkystonePosition1 == 2) {
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);
            robot1.autoArm.setPosition(0.4);
            movemento.strafe(-30, 0.5);
            movemento.straightMM(60, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-60, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.7);
            movemento.recalibrate(3, 90, 1);
            movemento.straightMM(555, 1);
            movemento.rotate(-80, 0.5);
            movemento.recalibrate(1, 10, 1);
            movemento.straightMM(110, 0.5);
            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.straightMM(-50, 0.5);
            movemento.rotate(-170, 0.5);
            movemento.straightMM(-100, 0.5);
            robot1.foundationgrabber.setPosition(0.975);
            sleep(1500);
            movemento.straightMM(230, 0.5);
            movemento.rotate(80, 0.5);
            robot1.foundationgrabber.setPosition(0.3);
            sleep(1000);
            movemento.straightMM(-50, 0.5);

            movemento.strafe(-100, 1);

            movemento.straightMM(300, 1);


        } else if (SkystonePosition1 == 3) {
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);
            robot1.autoArm.setPosition(0.4);
            movemento.strafe(-60, 0.5);
            movemento.straightMM(60, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-60, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(90, 0.7);
            movemento.recalibrate(3, 90, 1);
            movemento.straightMM(625, 1);
            movemento.rotate(-80, 0.5);
            movemento.recalibrate(1, 10, 1);
            movemento.straightMM(110, 0.5);
            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.straightMM(-50, 0.5);
            movemento.rotate(-170, 0.5);
            movemento.straightMM(-100, 0.5);
            robot1.foundationgrabber.setPosition(0.975);
            sleep(1500);
            movemento.straightMM(230, 0.5);
            movemento.rotate(80, 0.5);
            robot1.foundationgrabber.setPosition(0.3);
            sleep(1000);
            movemento.straightMM(-50, 0.5);

            movemento.strafe(-100, 1);

            movemento.straightMM(300, 1);


        }

    }



}