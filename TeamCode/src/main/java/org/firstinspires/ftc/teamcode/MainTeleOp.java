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
    private boolean ballInGondola = false;
    private boolean ballInIntake = false;
    private boolean vibrated = true;
    private boolean armInAuto = false;
    private boolean armOutAuto = false;
    private boolean backwardsFlag = false;
    private int intakeTimer = 0;
    private int backwardsTimer = 0;
    private boolean frontIntakeFlag;
    private boolean backIntakeFlag;
    private boolean frontIntake = true;
    private boolean robotBackwards = false;

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        Carousel.initCarousel(hardwareMap);
        Arm.initArm(hardwareMap);
        Intake.initIntake(hardwareMap);
        Auto.initAuto(hardwareMap);

        waitForStart();

        Intake.setFrontConstant();
        Intake.setBackConstant();

        while(opModeIsActive()){

            /*****DriveTrain*****/
            if(gamepad1.left_trigger > .2 && !robotBackwards){
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.cartesianDrive((-gamepad1.left_stick_x / 3), (gamepad1.left_stick_y / 3), (gamepad1.right_stick_x / 3));
            }else if(!robotBackwards) {
                DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            if(gamepad1.dpad_up){
                DriveTrain.resetGyro();
            }

            /*****Intake*****/
            //Intake forward
            if (gamepad2.x && !intakeFlagFoward && !ballInGondola && !backwardsFlag) {
                intakeFlagFoward = true;
                Intake.intakeChangeState("FORWARD");
            }
            //Intake backwards
            else if (gamepad2.b && !intakeFlagReverse /*&& !ballInGondola*/){
                intakeFlagReverse = true;
                Intake.intakeChangeState("REVERSE");
            }

            //Reset intakes flags
            if(intakeFlagFoward && !gamepad2.x){
                Intake.intakeChangeState("N/A");
                intakeFlagFoward = false;
            } else if(intakeFlagReverse && !gamepad2.b){
                Intake.intakeChangeState("N/A");
                intakeFlagReverse = false;
            }

            //Check for something in intake, change LEDs
            if((Intake.ballInFrontSensor() || Intake.ballInBackSensor()) && !ballInGondola && !ballInIntake && vibrated){
                ballInIntake = true;
                intakeTimer = 0;
                backwardsFlag = false;
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                robotBackwards = true;
                vibrated = false;
                if(Intake.ballInFrontSensor()) {
                    frontIntake = true;
                    DriveTrain.customDriveNoTimer(-.8, -.8, -.8, -.8);
                }
                else {
                    frontIntake = false;
                    DriveTrain.customDriveNoTimer(.8, .8, .8, .8);
                }
                Intake.frontIntakeUp();
                Intake.backIntakeUp();
            }

            //Start timer to intake backwards
            if(ballInIntake){
                intakeTimer++;
            }

            if(intakeTimer == 6){
                DriveTrain.customDriveNoTimer(0, 0, 0, 0);
                robotBackwards = false;
            }

            //When timer finishes, spin intake backwards
            if(intakeTimer > 13 && !backwardsFlag){
                Intake.changeIntakeBackwards();
                backwardsFlag = true;
                ballInIntake = false;
            }

            if(Arm.ballInGondola()){
                Intake.changeIntakeOff();
                ballInGondola = true;
                backwardsFlag = false;
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            //Change LEDs aqua if something in intake
            if(!ballInGondola && (Intake.ballInFrontSensor() || Intake.ballInBackSensor())){
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }

            /*****Carousel*****/
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

            /*****Arm*****/
            //Change arm speed
            if(gamepad2.left_bumper){
                Arm.changeSpeed("SLOW");
            }
            if(gamepad2.right_bumper){
                Arm.changeSpeed("FAST");
            }

            //Extend/retract arm fast
            if (gamepad2.right_trigger > .2 && !armFlagFast && !armInAuto && !armOutAuto) {
                armFlagFast = true;
                Arm.armChangeState("OUT");
            }
            //Extend/retract arm fast
            else if (gamepad2.left_trigger > .2 && !armFlagSlow && !armInAuto && !armOutAuto){
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

            if(gamepad2.a && !armInAuto){
                //Arm.heightChangeState("DOWN");
                armInAuto = true;
            }

            if(armInAuto){
                Arm.arm.setPower(-0.6);
            }

            if(Arm.getArmPos() < 10 && armInAuto){
                Arm.arm.setPower(0);
                Arm.changeArmIn();
                Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armInAuto = false;
            }

            if(gamepad2.y && !armOutAuto){
                Arm.heightChangeState("UP");
                armOutAuto = true;
            }

            if(armOutAuto){
                Arm.arm.setPower(0.6);
            }

            if(Arm.getArmPos() > 990 && armOutAuto){
                Arm.arm.setPower(0);
                Arm.changeArmOut();
                armOutAuto = false;
            }


            if(gamepad2.dpad_left) {
                Arm.releaseFreight();
                ballInGondola = false;
                backwardsFlag = false;
                ballInIntake = false;
                intakeTimer = 0;
                Intake.changeIntakeOff();
                vibrated = true;
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                if(frontIntake)
                    Intake.frontIntakeDown();
                else
                    Intake.backIntakeDown();
            }
            if(!Arm.ballInGondola()){
                ballInGondola = false;
            }

            /*****Arm Height*****/

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
            }

            //Reset height flag
            if(heightFlag && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down/*  && gamepad2.left_stick_y <= .3 && !gamepad2.dpad_left*/){
                heightFlag = false;
            }

            //Update States
            Carousel.carouselUpdatePosition(gamepad1.right_trigger);
            if(!armInAuto && !armOutAuto)
                Arm.armUpdatePosition();
            Intake.intakeUpdatePosition();
            if(!manualHeight)
                Arm.heightUpdatePosition();

            if(gamepad2.right_stick_y < -.2 && !backIntakeFlag){
                backIntakeFlag = true;
                Intake.backIntakeUp();
            } else if(gamepad2.right_stick_y > .2 && !backIntakeFlag){
                backIntakeFlag = true;
                Intake.backIntakeDown();
            }

            if(gamepad2.left_stick_y < -.2 && !frontIntakeFlag){
                frontIntakeFlag = true;
                Intake.frontIntakeUp();
            } else if(gamepad2.left_stick_y > .2 && !frontIntakeFlag){
                frontIntakeFlag = true;
                Intake.frontIntakeDown();
            }

            if(gamepad2.right_stick_y <= .2 && gamepad2.right_stick_y >= -.2){
                backIntakeFlag = false;
            }
            if(gamepad2.left_stick_y <= .2 && gamepad2.left_stick_y >= -.2){
                frontIntakeFlag = false;
            }

            if(gamepad1.dpad_right)
                Arm.cappingArm.setPosition(.73);
            else if(gamepad1.dpad_down)
                Arm.cappingArm.setPosition(.47);
            else if(gamepad1.dpad_left)
                Arm.cappingArm.setPosition(.1);

            if(gamepad1.a){
                if(armInAuto){
                    Arm.arm.setPower(0);
                    Arm.changeArmIn();
                    Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armInAuto = false;
                }
                else if(armOutAuto) {
                    Arm.arm.setPower(0);
                    Arm.changeArmIn();
                    armOutAuto = false;
                }
            }

            /*if(gamepad1.a)
                Arm.vibrator.setPosition(0);
            if(gamepad1.x)
                Arm.vibrator.setPosition(.4);
            if(gamepad1.y)
                Arm.vibrator.setPosition(.6);

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

            telemetry.addData("Back Servo pos: ", Intake.backServo.getPosition());
            telemetry.addData("Right stick y pos: ", gamepad2.right_stick_y);


            telemetry.addLine()
                    .addData("Intake back", " sensor")
                    .addData("Red", "%.3f", (double) Intake.intakeBackSensor.red())
                    .addData("Blue", "%.3f", (double) Intake.intakeBackSensor.blue())
                    .addData("Alpha", "%.3f", (double) Intake.intakeBackSensor.alpha());

            telemetry.addLine()
                    .addData("Wall color", " sensor")
                    .addData("Red", "%.3f", (double) DriveTrain.wallColorSensor.red())
                    .addData("Blue", "%.3f", (double) DriveTrain.wallColorSensor.blue())
                    .addData("Alpha", "%.3f", (double) DriveTrain.wallColorSensor.alpha());

            telemetry.addData("Left front: ", DriveTrain.leftFront.getCurrentPosition());


            telemetry.addData("Distance to gondola: ", Arm.armSensor.getDistance(DistanceUnit.CM));

            telemetry.addData("Distance to hub: ", Arm.gondolaSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Dead Wheel pos: ", Auto.getYPositon());
            telemetry.addData("Arm pos: ", Arm.getArmPos());

            DriveTrain.gyroTele(telemetry);



            telemetry.update();
            
             */
            telemetry.addData("Front constant: ", Intake.getFrontConstant());
            telemetry.addData("Back constant: ", Intake.getBackConstant());
            telemetry.update();
        }
    }
}
