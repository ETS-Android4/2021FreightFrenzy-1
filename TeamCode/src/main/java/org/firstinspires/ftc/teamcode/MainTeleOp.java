package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;

@TeleOp(name="Push Bot Test", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode{

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        //Carousel.initCarousel(hardwareMap);
        Arm.initArm(hardwareMap);

        waitForStart();

        boolean adjust = false;
        while(opModeIsActive()){

            DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.dpad_up){
                DriveTrain.resetGyro();
            }

            /*if(gamepad2.x){
                while(gamepad2.x){}
                if(adjust){
                    Carousel.reverseMotor();
                    adjust = !adjust;
                }else if(!adjust){
                    adjust = !adjust;
                }
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
            */

            if(gamepad1.y){
                Arm.moveArm(.6);
            }
            if(gamepad1.a){
                Arm.moveArm(.4);
            }



            //telemetry.addData("Adjust: ", adjust);
            //telemetry.addData("Carousel Power: ", Carousel.getCarouselPower());
            telemetry.update();
        }
    }
}
