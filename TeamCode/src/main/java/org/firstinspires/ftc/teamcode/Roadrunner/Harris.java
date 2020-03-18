package org.firstinspires.ftc.teamcode.Roadrunner.Tests;




import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RRHardware;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDriveREV;
import org.opencv.core.Mat;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Harris", group="LinearOpMode")
public class Harris extends LinearOpMode {
    RRHardware robot1 = new RRHardware();
    private ColorSensor sensorColor;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        robot1.init(hardwareMap);

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


        int SkystonePosition1 = 0;
        //0 is not found
        //1 is left side
        //2 is middle
        //3 is right

        int SkystonePossiblePosition = 1;

        boolean skystone = false;
        robot1.rgb.setPosition(0.92);


        telemetry.addLine("oy hya ya");
        telemetry.update();

        waitForStart();
        DriveConstraints constraints = new DriveConstraints(20, 40, 80, 1, 2, 4);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(path, constraints);



    }


}