package org.firstinspires.ftc.teamcode.Meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardWare1;
import org.firstinspires.ftc.teamcode.Movement1;

@Autonomous(name="Auto Yeet 3 Blue Foundation", group="Linear Opmode")
public class Meet3BlueFoundation extends LinearOpMode {

    // Declare OpMode members(motors, servos, and sensors).
    HardWare1 robot1 = new HardWare1();
    Movement1 movemento = new Movement1();



    @Override
    public void runOpMode() {

        // you know that hardware file, here it is being called
        robot1.init(hardwareMap);
        movemento.init(hardwareMap, telemetry);
        robot1.foundationgrabber.setPosition(0.3);
        robot1.swipeServo.setPosition(0.3);
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


        waitForStart();
        movemento.rotate(10, 0.5);
        movemento.straightMM(-300, 0.5);
        robot1.foundationgrabber.setPosition(0.975);
        sleep(1000);
        movemento.straightMM(250, 0.5);
        movemento.rotate(80, 0.5);
        robot1.foundationgrabber.setPosition(0.3);
        sleep(1000);
        movemento.straightMM(-100, 0.5);

        movemento.strafe(25, 0.5);
        movemento.straightMM(300, 0.5);
    }



}