package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="Push Bot Test", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode{

    private boolean intakeFlagFoward = false;
    private boolean intakeFlagReverse = false;

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        //Carousel.initCarousel(hardwareMap);
        //Arm.initArm(hardwareMap);
        //Intake.initIntake(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.dpad_up){
                DriveTrain.resetGyro();
            }

            if(gamepad1.a) {
                DriveTrain.leftFront.setPower(.8);
                DriveTrain.leftBack.setPower(.8);
                DriveTrain.rightFront.setPower(.8);
                DriveTrain.rightBack.setPower(.8);
            }
            if(gamepad1.b) {
                DriveTrain.leftFront.setPower(.2);
                DriveTrain.leftBack.setPower(.2);
                DriveTrain.rightFront.setPower(.2);
                DriveTrain.rightBack.setPower(.2);
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

            /*if(gamepad1.y){
                Arm.moveArm(.6);
            }
            if(gamepad1.a){
                Arm.moveArm(.4);
            }

            //Intake forward
            if (gamepad1.x && !intakeFlagFoward) {
                intakeFlagFoward = true;
                Intake.intakeChangeState("FORWARD");
            }
            //Intake backwards
            else if (gamepad1.b && !intakeFlagReverse){
                intakeFlagReverse = true;
                Intake.intakeChangeState("REVERSE");
            }

            //Reset intakes flags
            if(intakeFlagFoward && !gamepad1.x){
                intakeFlagFoward = false;
            } else if(intakeFlagReverse && !gamepad1.b){
                intakeFlagReverse = false;
            }

            Intake.intakeUpdatePosition();

             */

            //telemetry.addData("Adjust: ", adjust);
            //telemetry.addData("Carousel Power: ", Carousel.getCarouselPower());
            //DriveTrain.composeTelemetry(telemetry);
            telemetry.addData("Left front encoder: ", DriveTrain.leftFront.getCurrentPosition());
            telemetry.addData("Left back encoder: ", DriveTrain.leftBack.getCurrentPosition());
            telemetry.addData("Right front encoder: ", DriveTrain.rightFront.getCurrentPosition());
            telemetry.addData("Right back encoder: ", DriveTrain.rightBack.getCurrentPosition());

            DriveTrain.gyroTele(telemetry);
            telemetry.update();
        }
    }
}
