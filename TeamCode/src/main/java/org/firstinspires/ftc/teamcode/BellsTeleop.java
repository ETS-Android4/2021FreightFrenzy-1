package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Bells;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="BellsTeleop", group="Linear Opmode")

public class BellsTeleop extends LinearOpMode{

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        //Carousel.initCarousel(hardwareMap);
        Bells.initBells(hardwareMap);

        waitForStart();

        double power = 0;
        boolean spin = false;
        while(opModeIsActive()){

            DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.dpad_up){
                DriveTrain.resetGyro();
            }

            if(gamepad1.a) {
                //while(gamepad1.a){}
                //Bells.bellsChangeState();
                power = -.7;
            }

            if(gamepad1.y) {
                //while(gamepad1.y){}
                //Bells.bellsChangeState();
                power = .7;
            }
            if(gamepad1.b) {
                //while(gamepad1.y){}
                //Bells.bellsChangeState();
                power = 0.08;
            }
//            if(gamepad1.x) {
//                //while(gamepad1.y){}
//                //Bells.bellsChangeState();
//                power = 0.07;
//            }

            Bells.spinBells(power);


            telemetry.addData("Actual Power: ", Bells.getPower());
            telemetry.addData("Bells Power: ", power);
            telemetry.update();
        }
    }
}
