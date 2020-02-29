package org.firstinspires.ftc.teamcode.Roadrunner.Tests;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
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


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(29)
                        .build()
        );
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

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeTo(new Vector2d(29, -15))

                                    .build()
                    );
                }  else if (SkystonePossiblePosition == 2) {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeTo(new Vector2d(26, -25))
                                    .build()
                    );
                    SkystonePossiblePosition = 3;
                    SkystonePosition1 = 3;
                    skystone = true;
                }
            }
        }
        LinearInterpolator interp = new LinearInterpolator(
                Math.toRadians(0), Math.toRadians(-45)
        );
        LinearInterpolator right45 = new LinearInterpolator(
                Math.toRadians(0), Math.toRadians(45)
        );

        LinearInterpolator forward = new LinearInterpolator(
                Math.toRadians(0), Math.toRadians(0)
        );
        LinearInterpolator right = new LinearInterpolator(
                Math.toRadians(0), Math.toRadians(90)
        );

        if (SkystonePosition1 == 1) {
            robot1.grabbythingy.setPosition(0.15);
            robot1.capstonedropper.setPosition(0);

            robot1.Squishy1.setPower(1);
            robot1.Squishy1.setPower(-1);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(27, 25), interp)
                            .lineTo(new Vector2d(35, -10))
                            .reverse()
                            .lineTo(new Vector2d(20, 20))

                            .build()


            );




            /*
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.lineTo(new Vector2d(27, 10), left)
                            //.lineTo(new Vector2d(27, 50), interp)
                            .lineTo(new Vector2d(27, 80), right)
                            .lineTo(new Vector2d(32, 80))
                            .build()
            );
            robot1.autoArm.setPosition(0.25);

             */

        }
        /*LineSegment line = new LineSegment(
                new Vector2d(0, 0),
                new Vector2d(56, 24)
        );
        LinearInterpolator interp = new LinearInterpolator(
                Math.toRadians(0), Math.toRadians(-90)
        );
        PathSegment segment = new PathSegment(line, interp);
        Path path = new Path(segment);
        if (isStopRequested()) return;
        Path path1 = new PathBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Pose2d(15, 15, 0))
                .lineTo(new Vector2d(30, 15))

                .build();



         */

    }


}