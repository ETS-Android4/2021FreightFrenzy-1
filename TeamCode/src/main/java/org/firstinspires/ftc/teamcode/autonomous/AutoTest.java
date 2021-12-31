package org.firstinspires.ftc.teamcode.autonomous;

import android.drm.DrmInfoEvent;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
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

        waitForStart();

        DriveTrain.driveToLine(.22,"WHITE", telemetry);

        Intake.intake();

        DriveTrain.cartesianDriveIntake(0, .3, 10, 50, telemetry);

        sleep(50);

        if(Arm.getArmSensorLength() > 8){
            DriveTrain.gyroTurnIntake(-Math.PI / 6, 100, 10);
        }

        Intake.setBackwards();

        DriveTrain.cartesianDriveTimer(-.4, -.2, 25);

        Intake.stop();

        DriveTrain.driveToLine(-.22, "WHITE", telemetry);

        DriveTrain.cartesianDriveTimer(0, -.3, 35);
        sleep(100);

        Arm.armUp();

        Arm.armOutUp();

        /*
        while(opModeIsActive()) {
            DriveTrain.SUMO_MODE();
        }
        */
    }
}
