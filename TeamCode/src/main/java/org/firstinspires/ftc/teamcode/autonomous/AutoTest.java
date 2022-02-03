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

        waitForStart();

        Auto.driveWallColor(-.5, telemetry);

        Auto.autoBrake(50);

        //Auto.goToPosition(10 * Constants.COUNTS_PER_INCH, .15, 250, telemetry, opModeIsActive());

        sleep(1000);
    }
}
