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
    private boolean armFlagFast = false;
    private boolean armFlagSlow = false;
    private boolean heightFlag = false;

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



            //Intake forward
            if (gamepad2.x && !intakeFlagFoward) {
                intakeFlagFoward = true;
                Intake.intakeChangeState("FORWARD");
            }
            //Intake backwards
            else if (gamepad2.b && !intakeFlagReverse){
                intakeFlagReverse = true;
                Intake.intakeChangeState("REVERSE");
            }

            //Reset intakes flags
            if(intakeFlagFoward && !gamepad1.x){
                intakeFlagFoward = false;
            } else if(intakeFlagReverse && !gamepad1.b){
                intakeFlagReverse = false;
            }



            /*
            //Change carousel state
            if (gamepad2.x && !carouselFlag) {
                carouselFlag = true;
                Carousel.carouselChangeState();
            }

            //Reset carousel flags
            if(carouselFlag && !gamepad2.x){
                carouselFlag = false;
            }
            */

            //Change carousel color
            if(gamepad1.left_bumper){
                Carousel.changeColor("BLUE");
            }
            if(gamepad1.right_bumper){
                Carousel.changeColor("RED");
            }

            //Change carousel state
            if(gamepad1.right_trigger > .2 && !carouselFlag){
                carouselFlag = true;
                Carousel.carouselChangeState();
            }

            //Reset carousel flags
            if(carouselFlag && gamepad1.right_trigger <= .2){
                Carousel.carouselChangeState();
                carouselFlag = false;
            }



            //Extend/retract arm fast
            if (gamepad2.y && !armFlagFast) {
                armFlagFast = true;
                Arm.armChangeState("FAST");
            }
            //Extend/retract arm fast
            else if (gamepad2.a && !armFlagSlow){
                armFlagSlow = true;
                Arm.armChangeState("SLOW");
            }

            //Reset arm flags
            if(armFlagFast && !gamepad1.y){
                armFlagFast = false;
            }
            if(armFlagSlow && !gamepad1.a){
                armFlagSlow = false;
            }



            //Raise and lower arm
            if(gamepad2.dpad_up && !heightFlag){
                heightFlag = true;
                Arm.heightChangeState();
            }

            //Reset height flag
            if(heightFlag && !gamepad1.dpad_right){
                heightFlag = false;
            }



            //Update States
            Carousel.carouselUpdatePosition(gamepad1.right_trigger);
            Arm.armUpdatePosition();
            Arm.heightUpdatePosition();
            Intake.intakeUpdatePosition();

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
