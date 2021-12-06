package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(name= "AutoTest", group= "Autonomous")

public class AutoTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);

        waitForStart();

        DriveTrain.cartesianDriveTimer(0, .3, 40);

        sleep(100);

        DriveTrain.gyroTurn(-Math.PI / 2, 100);

        sleep(100);

        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

        while(opModeIsActive()) {
            DriveTrain.SUMO_MODE();
        }

    }
}
