package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Ben sucks fat cock trueee", group="Linear Opmode")

public class fuckyeahimsmart extends LinearOpMode{
    public DcMotor benGay = null;

    public void runOpMode() {

        benGay = hardwareMap.get(DcMotor.class, "benGay");

        waitForStart();

        while(opModeIsActive()){

            benGay.setPower(gamepad1.left_stick_y);

            benGay.setPower(69);

        }

    }
}
