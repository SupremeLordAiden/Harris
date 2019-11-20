package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import org.firstinspires.ftc.robotcore.external.State;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Outreach", group="Linear Opmode")


public class Outreach extends LinearOpMode {
    HardWare1 robot1 = new HardWare1();
    public ElapsedTime mRunTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Made by Calvin");
        telemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot1.init(hardwareMap);
        waitForStart();
        robot1.leftDrive.setPower(1);
        robot1.rightDrive.setPower(-1);
        sleep(500);
        robot1.leftDrive.setPower(0);
        robot1.rightDrive.setPower(0);
        sleep(1000);
        robot1.leftDrive.setPower(-1);
        robot1.rightDrive.setPower(1);
        sleep(1000);
        while (mRunTime.time() < 10000) {
            telemetry.addLine("spinning");
            telemetry.update();
            robot1.leftDrive.setPower(1);
            robot1.rightDrive.setPower(1);
        }
    }
}