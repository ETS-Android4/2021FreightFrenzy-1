package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;

@TeleOp(name="Push Bot Test", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode{

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        Carousel.initCarousel(hardwareMap);

        waitForStart();

        boolean adjust = false;
        while(opModeIsActive()){

            DriveTrain.leftFront.setPower(gamepad1.left_stick_y);
            DriveTrain.rightFront.setPower(gamepad1.right_stick_y);

            if(gamepad2.x){
                adjust = !adjust;
            }

            if(gamepad2.y) {
                while(gamepad2.y){}
                Carousel.adjustUp();
            }else if(gamepad2.a) {
                while(gamepad2.a){}
                Carousel.adjustDown();
            }

            if(adjust) {
                Carousel.powerMotor();
            }else {
                Carousel.accelerate(gamepad2.left_stick_y);
            }

            telemetry.addData("Adjust: ", adjust);
            telemetry.addData("Carousel Power: ", Carousel.carouselPower);
            telemetry.update();
        }
    }
}
