package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Harris Testing", group="Linear Opmode")
public class HarrisTestingAuto extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).
    HardWare1 robot1 = new HardWare1();
    Movement1 movemento = new Movement1();

    private ColorSensor sensorColor;


    int SkystonePosition1 = 0;
    //0 is not found
    //1 is left side
    //2 is middle
    //3 is right



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

        robot1.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot1.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot1.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot1.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("oy hya ya");
        telemetry.update();

        waitForStart();
        robot1.capstonedropper.setPosition(1);
        movemento.pidStraight(100, 0.5);
        //movemento.pidRotate(135, 1);
        //movemento.pidRotate(-135, 1);
        //movemento.pidRotate(90, 1);
        //movemento.pidRotate(-90, 1);
    }



}