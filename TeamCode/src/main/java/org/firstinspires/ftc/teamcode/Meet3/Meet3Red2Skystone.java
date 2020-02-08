package org.firstinspires.ftc.teamcode.Meet3;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardWare1;
import org.firstinspires.ftc.teamcode.Movement1;

@Autonomous(name="Auto Yeet 3 Red 2 Skystone", group="Linear Opmode")


public class Meet3Red2Skystone extends LinearOpMode {

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

        robot1.autoPush2.setPosition(1);
        robot1.autoArm2.setPosition(1);


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
        robot1.rgb.setPosition(0.81);
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
                movemento.strafe(-75, 0.5);
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
            robot1.autoArm.setPosition(0.39);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-10, 0.4);


            sleep(1000);
            movemento.straightMM(30, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-90, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(-90, 0.6);
            movemento.recalibrate(1, -90, 1);
            movemento.straightMM(380, 1);

            robot1.autoArm.setPosition(0.3);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);

            movemento.recalibrate(1, -90, 1);


            movemento.straightMM(-540, 0.75);

            robot1.autoArm.setPosition(0.385);
            movemento.rotate(90, 0.5);
            movemento.forwardMMwithDistanceRight(90, 0.3, 10);

            robot1.autoPush.setPosition(0);
            sleep(1000);

            movemento.straightMM(-90,0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(-100, 0.5);
            movemento.recalibrate(1, -90, 1);
            movemento.straightMM(540, 1);
            robot1.autoArm.setPosition(0.3);
            robot1.autoPush.setPosition(0.5);
            sleep(500);
            movemento.straightMM(-150, 1);


        } else if (SkystonePosition1 == 2) {
            robot1.autoArm.setPosition(0.39);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);
            movemento.strafe(-20, 0.5);
            movemento.straightMM(50, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-80, 0.5);
            robot1.autoArm.setPosition(0.3);
            movemento.rotate(-90, 0.4);
            movemento.recalibrate(1, -90, 1);
            movemento.straightMM(480, 1);

            robot1.autoArm.setPosition(0.3);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);

            movemento.recalibrate(1, -90, 1);


            movemento.straightMM(-540, 0.75);

            robot1.autoPush2.setPosition(0.5);
            robot1.autoArm2.setPosition(0.6);
            movemento.rotate(90, 0.5);

            movemento.forwardMMwithDistanceLeft(100, 0.3, 10);
            robot1.autoPush2.setPosition(0.9);
            sleep(1000);
            movemento.straightMM(-70, 0.5);
            robot1.autoArm2.setPosition(0.7);
            movemento.rotate(-90, 0.5);
            movemento.recalibrate(1, -90, 1);
            movemento.straightMM(550,1);
            robot1.autoPush2.setPosition(0);
            robot1.autoArm2.setPosition(0.65);
            sleep(1000);
            movemento.straightMM(-150, 0.5);



        } else if (SkystonePosition1 == 3) {

            robot1.autoArm.setPosition(0.39);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);

            movemento.strafe(-20, 0.5);
            movemento.straightMM(50, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-100, 0.5);
            robot1.autoArm.setPosition(0.3);
            movemento.rotate(-90, 0.6);
            movemento.recalibrate(1, -90, 1);
            movemento.straightMM(520, 1);

            robot1.autoArm.setPosition(0.39);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.recalibrate(1, -90, 1);


            movemento.straightMM(-600, 0.75);
            robot1.autoPush2.setPosition(0.5);
            robot1.autoArm2.setPosition(0.6);
            movemento.strafe(-70, 0.5);
            robot1.capstonedropper.setPosition(0.5);

            movemento.rotate(90, 0.5);
            movemento.wallStrafe(1, 0.5);
            movemento.strafe(10, 0.5);
            movemento.forwardMMwithDistanceLeft(80, 0.5, 10);
            robot1.autoPush2.setPosition(0.9);
            sleep(1000);
            movemento.straightMM(-70, 0.5);
            robot1.autoArm2.setPosition(0.7);
            movemento.strafe(50, 0.5);
            movemento.rotate(-90, 0.5);
            movemento.recalibrate(1, -90, 1);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(650,1);
            robot1.autoPush2.setPosition(0);
            robot1.autoArm2.setPosition(0.65);
            movemento.straightMM(-150, 0.3);


        }

    }



}