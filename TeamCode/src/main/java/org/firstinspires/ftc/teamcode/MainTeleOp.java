package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="Push Bot Test", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode{

    private boolean intakeFlagFoward = false;
    private boolean intakeFlagReverse = false;
    private boolean carouselFlag = false;

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        Carousel.initCarousel(hardwareMap);
        //Arm.initArm(hardwareMap);
        //Intake.initIntake(hardwareMap);

        boolean adjust = false;
        waitForStart();

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

            //Change carousel color
            if(gamepad2.left_bumper){
                Carousel.changeColor("BLUE");
            }
            if(gamepad2.right_bumper){
                Carousel.changeColor("RED");
            }

            //Change carousel state
            if (gamepad2.x && !carouselFlag) {
                carouselFlag = true;
                Carousel.carouselChangeState();
            }

            //Reset carousel flags
            if(carouselFlag && !gamepad2.x){
                carouselFlag = false;
            }


            //Update Carousel State
            Carousel.carouselUpdatePosition();

            //telemetry.addData("Adjust: ", adjust);
            telemetry.addData("Carousel Power: ", Carousel.getCarouselPower());
            //DriveTrain.composeTelemetry(telemetry);
            telemetry.addData("Left front encoder: ", DriveTrain.leftFront.getCurrentPosition());
            telemetry.addData("Left back encoder: ", DriveTrain.leftBack.getCurrentPosition());
            telemetry.addData("Right front encoder: ", DriveTrain.rightFront.getCurrentPosition());
            telemetry.addData("Right back encoder: ", DriveTrain.rightBack.getCurrentPosition());
            telemetry.addData("Color: ", Carousel.getColor());

            telemetry.addData("Carousel state: ", Carousel.getState());

            telemetry.addData("tower distance", DriveTrain.towerSensor.getDistance(DistanceUnit.CM));

            DriveTrain.gyroTele(telemetry);
            telemetry.update();
        }
    }
}
