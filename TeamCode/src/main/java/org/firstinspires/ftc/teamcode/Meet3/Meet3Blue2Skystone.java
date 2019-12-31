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
        movemento.init(hardwareMap, telemetry);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "color");


        robot1.foundationgrabber.setPosition(0.3);
        robot1.swipeServo.setPosition(0.925);
        robot1.grabbythingy.setPosition(0.8);
        robot1.armthingy.setPosition(0.75);
        robot1.capstonedropper.setPosition(0.5);

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

        movemento.forwardMMwithDistance(230, 0.5, 10);
        //movemento.straightMM(220, 0.5);

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
                    SkystonePossiblePosition = 3;
                    SkystonePosition1 = 3;
                    skystone = true;
                }
            }
        }
        if (SkystonePosition1 == 1) {
            robot1.capstonedropper.setPosition(0);
            robot1.grabbythingy.setPosition(0.15);
            movemento.strafe(170, 0.5);

            movemento.rotate(90, 0.5);

            movemento.strafe(130, 0.5);
            robot1.Squishy1.setPower(-0.6);
            robot1.Squishy2.setPower(0.6);

            movemento.straightMM(80, 0.5);
            robot1.Squishy1.setPower(0);
            robot1.Squishy2.setPower(0);
            movemento.strafe(-150, 0.5);


            movemento.straightMM(400, 1);
            robot1.Squishy1.setPower(0.6);
            robot1.Squishy2.setPower(-0.6);
            movemento.straightMM(-100, 1);

        } else if (SkystonePosition1 == 2)  {
            robot1.capstonedropper.setPosition(0);
            robot1.grabbythingy.setPosition(0.15);
            movemento.strafe(-110, 0.5);

            movemento.rotate(-90, 0.5);

            movemento.strafe(-130, 0.5);

            movemento.recalibrate(2, -90, 1);

            robot1.Squishy1.setPower(-1);
            robot1.Squishy2.setPower(1);

            movemento.straightMM(80, 0.5);
            robot1.Squishy1.setPower(0);
            robot1.Squishy2.setPower(0);
            movemento.strafe(150, 0.5);



            movemento.straightMM(-400, 1);

            movemento.rotate(90, 0.5);
            robot1.Squishy1.setPower(0.6);
            robot1.Squishy2.setPower(-0.6);
            movemento.straightMM(10, 0.5);
            movemento.straightMM(-10, 0.5);
            movemento.rotate(-90, 0.5);
            movemento.recalibrate(4, -90, 0.5);
            movemento.straightMM(580, 1);
            movemento.strafe(-230, 0.5);
            movemento.recalibrate(2, -90, 0.5);
            robot1.Squishy1.setPower(-0.6);
            robot1.Squishy2.setPower(0.6);

            movemento.straightMM(80, 0.5);
            robot1.Squishy1.setPower(0);
            robot1.Squishy2.setPower(0);
            movemento.strafe(160, 0.5);
            movemento.recalibrate(4, -90, 0.5);

            movemento.straightMM(-550, 1);
            movemento.rotate(90, 0.5);
            robot1.Squishy1.setPower(0.6);
            robot1.Squishy2.setPower(-0.6);
            movemento.straightMM(10, 0.4);
            movemento.straightMM(-10, 0.4);
            movemento.strafe(200, 0.5);

        }
        else if (SkystonePosition1 == 3)  {
            robot1.capstonedropper.setPosition(0);
            robot1.grabbythingy.setPosition(0.15);
            movemento.strafe(-20, 0.5);

            movemento.rotate(-90, 0.5);

            movemento.strafe(-130, 0.5);

            movemento.recalibrate(1, -90, 1);

            robot1.Squishy1.setPower(-1);
            robot1.Squishy2.setPower(1);

            movemento.straightMM(80, 0.5);
            robot1.Squishy1.setPower(0);
            robot1.Squishy2.setPower(0);
            movemento.strafe(150, 0.5);



            movemento.straightMM(-400, 1);

            movemento.rotate(90, 0.5);
            robot1.Squishy1.setPower(0.6);
            robot1.Squishy2.setPower(-0.6);
            movemento.straightMM(10, 0.5);
            movemento.straightMM(-10, 0.5);
            movemento.rotate(-90, 0.5);
            movemento.recalibrate(4, -90, 0.5);
            movemento.straightMM(600, 1);
            movemento.strafe(-230, 0.5);
            movemento.recalibrate(2, -90, 0.5);
            robot1.Squishy1.setPower(-0.6);
            robot1.Squishy2.setPower(0.6);

            movemento.straightMM(80, 0.5);
            robot1.Squishy1.setPower(0);
            robot1.Squishy2.setPower(0);
            movemento.strafe(180, 0.5);
            movemento.recalibrate(4, -90, 0.5);

            movemento.straightMM(-650, 1);
            movemento.rotate(90, 0.5);
            robot1.Squishy1.setPower(0.6);
            robot1.Squishy2.setPower(-0.6);
            movemento.straightMM(10, 0.4);
            movemento.straightMM(-10, 0.4);
            movemento.strafe(200, 0.5);
        }
    }



}