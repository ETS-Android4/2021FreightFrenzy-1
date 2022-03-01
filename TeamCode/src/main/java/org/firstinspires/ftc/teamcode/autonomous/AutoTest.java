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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "AutoTest", group= "Autonomous")

public class AutoTest extends LinearOpMode {

    public static ElapsedTime timeyBoi = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Intake.initIntake(hardwareMap);
        Arm.initArm(hardwareMap);
        Carousel.initCarousel(hardwareMap);
        Auto.initAuto(hardwareMap);


        waitForStart();

        super.resetStartTime();

        while(super.getRuntime() < 25){
            telemetry.addData("Run time: ", super.getRuntime());
            telemetry.update();
        }

        stop();

    }
}
