package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="Push Bot Test", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode{

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            DriveTrain.leftFront.setPower(gamepad1.left_stick_y);
            DriveTrain.rightFront.setPower(gamepad1.right_stick_y);

        }
    }
}
