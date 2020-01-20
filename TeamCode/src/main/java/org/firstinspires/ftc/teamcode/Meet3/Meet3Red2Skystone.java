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
            robot1.autoArm.setPosition(0.5);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-10, 0.4);

            movemento.strafe(-30, 0.5);
            movemento.straightMM(30, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-70, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(-90, 0.7);
            movemento.recalibrate(2, -90, 0.5);
            movemento.straightMM(380, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.recalibrate(2, -90, 0.5);


            movemento.straightMM(-580, 0.75);

            robot1.autoArm.setPosition(0.4);
            movemento.rotate(100, 0.5);
            movemento.straightMM(85, 0.2);

            robot1.autoPush.setPosition(0);
            sleep(1000);

            movemento.straightMM(-70,0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(-100, 0.5);
            movemento.recalibrate(2, -90, 0.5);
            movemento.straightMM(640, 0.5);
            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);
            movemento.straightMM(-100, 0.5);


        } else if (SkystonePosition1 == 2) {
            robot1.autoArm.setPosition(0.4);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);

            movemento.strafe(-30, 0.5);
            movemento.straightMM(50, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-80, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(-90, 0.7);
            movemento.recalibrate(2, -90, 0.5);
            movemento.straightMM(480, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.recalibrate(2, -90, 0.5);


            movemento.straightMM(-180, 0.75);




        } else if (SkystonePosition1 == 3) {

            robot1.autoArm.setPosition(0.4);
            robot1.capstonedropper.setPosition(0);
            movemento.straightMM(-30, 0.4);

            movemento.strafe(-50, 0.5);
            movemento.straightMM(50, 0.5);
            robot1.autoPush.setPosition(0);
            sleep(1000);
            movemento.straightMM(-80, 0.5);
            robot1.autoArm.setPosition(0.25);
            movemento.rotate(-90, 0.7);
            movemento.recalibrate(2, -90, 0.5);
            movemento.straightMM(520, 1);

            robot1.autoArm.setPosition(0.4);
            robot1.autoPush.setPosition(0.5);
            sleep(500);

            robot1.autoPush.setPosition(0.6);
            robot1.autoArm.setPosition(0);
            movemento.recalibrate(2, -90, 0.5);


            movemento.straightMM(-150, 0.75);


        }

    }



}