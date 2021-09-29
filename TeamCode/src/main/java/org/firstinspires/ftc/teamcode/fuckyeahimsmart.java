package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Push Bot Test", group="Linear Opmode")

public class fuckyeahimsmart extends LinearOpMode{
    public DcMotor frontMotor;
    public DcMotor backMotor;

    public void runOpMode() {

        frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        backMotor = hardwareMap.get(DcMotor.class, "backMotor");

        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            frontMotor.setPower(gamepad1.right_stick_y);
            backMotor.setPower(gamepad1.left_stick_y);

        }

    }
}
