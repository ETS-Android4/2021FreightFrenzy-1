package org.firstinspires.ftc.teamcode.bells;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bells.Bells;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="BellsTeleop", group="Linear Opmode")
@Disabled
public class BellsTeleop extends LinearOpMode{

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        Bells.initBells(hardwareMap);

        waitForStart();

        double power = 0;
        while(opModeIsActive()){

            DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.dpad_up){
                DriveTrain.resetGyro();
            }

            if(gamepad1.a) {
                power = -.7;
            }

            if(gamepad1.y) {
                power = .7;
            }
            if(gamepad1.b) {
                power = 0.08;
            }

            Bells.spinBells(power);

            telemetry.addData("Actual Power: ", Bells.getPower());
            telemetry.addData("Bells Power: ", power);
            telemetry.update();
        }
    }
}
