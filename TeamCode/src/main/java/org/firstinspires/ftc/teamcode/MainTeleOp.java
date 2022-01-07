package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="TeleOop", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode{

    private boolean intakeFlagFoward = false;
    private boolean intakeFlagReverse = false;
    private boolean carouselFlag = false;
    private boolean armFlagFast = false;
    private boolean armFlagSlow = false;
    private boolean armFlag = false;
    private boolean heightFlag = false;
    private boolean manualHeight = false;
    private boolean manualHeightFlag = false;
    private boolean ballInGondola = false;
    private boolean ballInIntake = false;
    private boolean autoVibrate = false;
    private boolean vibrated = false;
    private boolean armInAuto = false;
    private int intakeTimer = 0;

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        Carousel.initCarousel(hardwareMap);
        Arm.initArm(hardwareMap);
        Intake.initIntake(hardwareMap);
        Auto.initAuto(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.dpad_up){
                DriveTrain.resetGyro();
            }

            /*if(Arm.arm.getCurrentPosition() > 500 && Arm.gondolaSensor.getDistance(DistanceUnit.CM) < 35 && !vibrated){
                Arm.releaseFreight();
                ballInGondola = false;
                Intake.changeIntakeOff();
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                Arm.stopArm();
                autoVibrate = true;
                vibrated = true;
            }*/

            if(Arm.armIsIn())
                vibrated = false;

            if((Arm.ballInGondola() || Intake.ballInFrontSensor()) && !ballInGondola && !ballInIntake){
                ballInIntake = true;
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }

            if(ballInIntake){
                intakeTimer++;
            }

            if(intakeTimer > 1 && Arm.ballInGondola()){
                Intake.changeIntakeBackwards();
                ballInGondola = true;
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }else{
                ballInIntake = false;
            }

            if(ballInGondola){
                Intake.changeIntakeBackwards();
            }

            if(!ballInGondola && (Intake.ballInFrontSensor() || Intake.ballInBackSensor())){
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }

            //Intake forward
            if (gamepad2.x && !intakeFlagFoward && !ballInGondola) {
                intakeFlagFoward = true;
                Intake.intakeChangeState("FORWARD");
            }
            //Intake backwards
            else if (gamepad2.b && !intakeFlagReverse && !ballInGondola){
                intakeFlagReverse = true;
                Intake.intakeChangeState("REVERSE");
            }

            //Reset intakes flags
            if(intakeFlagFoward && !gamepad2.x){
                Intake.intakeChangeState("FORWARD");
                intakeFlagFoward = false;
            } else if(intakeFlagReverse && !gamepad2.b){
                Intake.intakeChangeState("REVERSE");
                intakeFlagReverse = false;
            }



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


            //Change arm speed
            if(gamepad2.left_bumper){
                Arm.changeSpeed("SLOW");
            }
            if(gamepad2.right_bumper){
                Arm.changeSpeed("FAST");
            }

            //Extend/retract arm fast
            if (gamepad2.right_trigger > .2 && !armFlagFast && !autoVibrate && !armInAuto) {
                armFlagFast = true;
                Arm.armChangeState("OUT");
            }
            //Extend/retract arm fast
            else if (gamepad2.left_trigger > .2 && !armFlagSlow && !autoVibrate && !armInAuto){
                armFlagSlow = true;
                Arm.armChangeState("IN");
            }

            //Reset arm flags
            if(armFlagFast && gamepad2.right_trigger <= .2){
                armFlagFast = false;
                Arm.armChangeState("N/A");
            }
            if(armFlagSlow && gamepad2.left_trigger <= .2){
                armFlagSlow = false;
                Arm.armChangeState("N/A");
            }

            if(gamepad2.left_trigger <= .2 && gamepad2.right_trigger <= .2){
                autoVibrate = false;
            }

            if(gamepad2.a && !armInAuto){
                Arm.armInTele();
            }

            if(Arm.getArmPos() < 0){
                Arm.arm.setPower(0);
                Arm.changeArmIn();
                Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armInAuto = false;
            }

            //Arm Out Up
            /*if(gamepad2.y && !armFlag){
                armFlag = true;
                Arm.armChangeState("N/A");
            }

            if(armFlag && !gamepad2.y){
                armFlag = false;
            }*/



            //Raise and lower arm
            if(gamepad2.dpad_up && !heightFlag) {
                heightFlag = true;
                manualHeight = false;
                Arm.heightChangeState("UP");
            } else if(gamepad2.dpad_right && !heightFlag){
                heightFlag = true;
                manualHeight = false;
                Arm.heightChangeState("MID");
            } else if(gamepad2.dpad_down && !heightFlag){
                heightFlag = true;
                manualHeight = false;
                Arm.heightChangeState("DOWN");
            } /*else if(gamepad2.dpad_left && !heightFlag){
                heightFlag = true;
                Arm.heightChangeState("MAX");
            }*/
            else if(gamepad2.left_stick_y > .3 && !heightFlag){
                heightFlag = true;
                manualHeight = false;
                Arm.heightChangeState("MAX");
            }

            //Reset height flag
            if(heightFlag && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down  && gamepad2.left_stick_y <= .3/* && !gamepad2.dpad_left*/){
                heightFlag = false;
            }

            if(gamepad2.dpad_left) {
                Arm.releaseFreight();
                ballInGondola = false;
                ballInIntake = false;
                intakeTimer = 0;
                Intake.changeIntakeOff();
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            if(gamepad2.right_stick_y < -.2 && !manualHeightFlag){
                if(Arm.heightServo1.getPosition() >= .2) {
                    manualHeightFlag = true;
                    manualHeight = true;
                    Arm.moveArm(Arm.heightServo1.getPosition() - .01);
                }
            } else if(gamepad2.right_stick_y > .2 && !manualHeightFlag){
                if(Arm.heightServo1.getPosition() <= .8) {
                    manualHeightFlag = true;
                    manualHeight = true;
                    Arm.moveArm(Arm.heightServo1.getPosition() + .01);
                }
            }

            if(gamepad2.right_stick_y <= .2 && gamepad2.right_stick_y >= -.2){
                manualHeightFlag = false;
            }

            if(gamepad1.dpad_left){
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            else if(gamepad1.dpad_right){
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }


            //Update States
            Carousel.carouselUpdatePosition(gamepad1.right_trigger);
            Arm.armUpdatePosition();
            Intake.intakeUpdatePosition();
            if(!manualHeight)
                Arm.heightUpdatePosition();

            /*
            //DriveTrain.composeTelemetry(telemetry);
            telemetry.addData("Left front encoder: ", DriveTrain.leftFront.getCurrentPosition());
            telemetry.addData("Left back encoder: ", DriveTrain.leftBack.getCurrentPosition());
            telemetry.addData("Right front encoder: ", DriveTrain.rightFront.getCurrentPosition());
            telemetry.addData("Right back encoder: ", DriveTrain.rightBack.getCurrentPosition());
            telemetry.addData("Color: ", Carousel.getColor());
            //telemetry.addData("Carousel state: ", Carousel.getState());
            telemetry.addData("arm state", Arm.getArmState());
            telemetry.addData("arm encoder pos: ", Arm.arm.getCurrentPosition());
            //telemetry.addData("arm speed: ", Arm.getSpeed());
            telemetry.addData("height servo pos: ", Arm.heightServo1.getPosition());
            DriveTrain.gyroTele(telemetry);
            */

            telemetry.addLine()
                    .addData("Floor color", " sensor")
                    .addData("Red", "%.3f", (double) DriveTrain.floorColorSensor.red())
                    .addData("Blue", "%.3f", (double) DriveTrain.floorColorSensor.blue())
                    .addData("Alpha", "%.3f", (double) DriveTrain.floorColorSensor.alpha());
            telemetry.addLine()
                    .addData("Intake front", " sensor")
                    .addData("Red", "%.3f", (double) Intake.intakeFrontSensor.red())
                    .addData("Blue", "%.3f", (double) Intake.intakeFrontSensor.blue())
                    .addData("Alpha", "%.3f", (double) Intake.intakeFrontSensor.alpha());
            telemetry.addLine()
                    .addData("Intake back", " sensor")
                    .addData("Red", "%.3f", (double) Intake.intakeBackSensor.red())
                    .addData("Blue", "%.3f", (double) Intake.intakeBackSensor.blue())
                    .addData("Alpha", "%.3f", (double) Intake.intakeBackSensor.alpha());
            telemetry.addData("Distance to gondola: ", Arm.armSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance to hub: ", Arm.gondolaSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Dead Wheel pos: ", Auto.getYPositon());

            DriveTrain.gyroTele(telemetry);

            telemetry.update();
        }
    }
}
