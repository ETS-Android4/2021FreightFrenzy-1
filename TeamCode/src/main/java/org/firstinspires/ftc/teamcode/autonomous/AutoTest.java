package org.firstinspires.ftc.teamcode.autonomous;

import android.drm.DrmInfoEvent;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name= "AutoTest", group= "Autonomous")

public class AutoTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Intake.initIntake(hardwareMap);
        Arm.initArm(hardwareMap);
        Carousel.initCarousel(hardwareMap);
        Auto.initAuto(hardwareMap);

        Arm.armUp();

        waitForStart();

        Arm.armOutUpFast();

        Auto.driveWallColor(-.2, telemetry);

        Auto.resetEncoder();

        Auto.goToPosition(-12 * Constants.COUNTS_PER_INCH, -.1, Constants.COUNTS_PER_INCH * 2, telemetry, opModeIsActive());

        //Auto.autoBrake(10);

        Arm.releaseFreight();

        Arm.armInNoReset();

        Auto.resetEncoder();

        Auto.goToPosition(39 * Constants.COUNTS_PER_INCH, .35, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        telemetry.addData("Encoder: ", Auto.getYPositon());
        telemetry.update();

        sleep(5000);
    }
}
