package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

@Autonomous(name= "BlueMainAuto", group= "Autonomous")

public class BlueMainAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
    private static final String VUFORIA_KEY = "AW/D0F3/////AAABmT6CO76ZukEWtNAvh1kty819QDEF9SG9ZxbfUcbjoxBCe0UcoTGK19TZdmHtWDwxhrL4idOt1tdJE+h9YGDtZ7U/njHEqSZ7jflzurT4j/WXTCjeTCSf0oMqcgduLTDNz+XEXMbPSlnHbO9ZnEZBun7HHr6N06kpYu6QZmG6WRvibuKCe5IeZJ21RoIeCsdp3ho/f/+QQLlnqaa1dw6i4xMFM0e2IaxujhQiWnd4by23CkMPvzKhy6YP3wPBq+awpzEPLDZcD8l1i0SqmX7HNpmw4kXBrWzEimAzp1aqONVau4kIwCGwJFusMdErw9IL7KQ5VqMKN4Xl67s0pwotoXsA+5SlWQAIodipYKZnPzwO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public String label;
    public boolean marker;
    public boolean arm;
    public boolean block;

    private static List<Recognition> tfodRecogntions;

    public static ElapsedTime timeyBoi = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        DriveTrain.initDriveTrain(hardwareMap);
        Arm.initArm(hardwareMap);
        Intake.initIntake(hardwareMap);
        Carousel.initCarousel(hardwareMap);
        Auto.initAuto(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        sleep(500);

        tfodRecogntions = tfod.getUpdatedRecognitions();

        for(Recognition recognition : tfodRecogntions) {
            telemetry.addData("Label: ", recognition.getLabel());
            if(recognition.getLabel().equals("Marker")) {
                marker = true;
            }else{
                marker = false;
            }
            if(!marker) {
                if (recognition.getLeft() < 135) {
                    label = "LEFT";
                } else if (recognition.getLeft() >= 135 && recognition.getLeft() <= 386) {
                    label = "MIDDLE";
                } else if (recognition.getLeft() > 386) {
                    label = "RIGHT";
                } else {
                    label = "None";
                }
            }
        }

        telemetry.addData("Element: ", label);
        telemetry.update();

        while(!isStarted()) {
            sleep(500);

            tfodRecogntions = tfod.getUpdatedRecognitions();

            for(Recognition recognition : tfodRecogntions) {
                telemetry.addData("Label: ", recognition.getLabel());
                if(recognition.getLabel().equals("Marker")) {
                    marker = true;
                }else{
                    marker = false;
                }
                if(!marker) {
                    if (recognition.getLeft() < 185) {
                        label = "LEFT";
                    } else if (recognition.getLeft() >= 185 && recognition.getLeft() <= 336) {
                        label = "MIDDLE";
                    } else if (recognition.getLeft() > 336) {
                        label = "RIGHT";
                    } else {
                        label = "None";
                    }
                }
            }

            telemetry.addData("Element: ", label);
            telemetry.update();
        }

        waitForStart();

        super.resetStartTime();

        Intake.frontIntakeDown();

        if(label == null || label.equals("RIGHT")) {
            Arm.armUp();

            sleep(900);

            Arm.armOutUpSlow();

            sleep(200);

            Arm.releaseFreightOpen();

            sleep(100);

            Arm.armDown();

            Arm.armInNoReset();
        } else if(label.equals("LEFT")){
            Arm.armDown();

            sleep(400);

            Arm.armOutDown();

            Arm.releaseFreightOpen();

            sleep(100);

            Arm.armInNoReset();
        }
        else if(label.equals("MIDDLE")){
            Arm.armMid();

            sleep(250);

            Arm.armOutMid();

            Arm.releaseFreightOpen();

            sleep(100);

            Arm.armDown();

            Arm.armInNoReset();
        }
        Arm.armUp();

        for(int i = 0; i < 5; i ++){
            if(super.getRuntime() <= 22){
                arm = false;
                block = false;
                Auto.resetEncoder();

                Intake.frontIntakeDown();

                Intake.intake();

                if(i == 0){
                    Auto.goToPositionIntake(45 * Constants.COUNTS_PER_INCH, .4, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
                }
                else{
                    Auto.goToPositionIntake((51 + (i * 2)) * Constants.COUNTS_PER_INCH, .4, 3 * Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
                }
                Arm.releaseFreightClose();

                Intake.setFrontConstant();
                Intake.setBackConstant();

                Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm.slideArm(-.25);

                int turn = 0;
                while(!block && super.getRuntime() < 28){
                    if(turn != 0)
                        Auto.driveIntakeColor(.20, 25, telemetry);
                    Auto.rotateColor(.35, 25, telemetry);
                    if(Intake.ballInFrontSensor())
                        block = true;
                    else if(!block)
                        DriveTrain.cartesianDriveTimer(.8, -.25, 15);
                    turn++;
                }

                if(super.getRuntime() <= 25){
                    Intake.frontIntakeUp();

                    Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Arm.slideArm(-.25);
                    Intake.intakeBack.setPower(.65);

                    Auto.driveWallColor(-.18, telemetry);

                    Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    if(Arm.ballInGondola()) {
                        arm = true;
                        Arm.armOutUpFast();
                    }

                    Auto.resetEncoder();

                    Auto.goToPosition(-4.5 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH * 3, telemetry, opModeIsActive());

                    if(!Arm.ballInGondola() && !arm) {
                        while (!Arm.ballInGondola()){
                            if(super.getRuntime() <= 28){}
                            else{
                                Arm.releaseFreightClose();
                                sleep(300);
                                break;
                            }
                        }
                        if(super.getRuntime() <= 28){
                            Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            Arm.armOutUp();
                        }
                        else{
                            Arm.releaseFreightClose();
                            sleep(300);
                            break;
                        }
                    }
                    else if(Arm.ballInGondola() && !arm){
                        Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Arm.armOutUp();
                    }
                    else if(arm){
                        while (Arm.getArmPos() < 1050);
                    }

                    Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Arm.stopArm();

                    Arm.releaseFreightOpen();

                    sleep(400);

                    Arm.armInNoReset();
                }
                else{
                    Arm.releaseFreightClose();
                    sleep(300);
                    stop();
                }
            }
            else{
                Arm.releaseFreightClose();
                sleep(300);
                break;
            }
        }

        Auto.goToPosition(49 * Constants.COUNTS_PER_INCH, .4, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        Arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.slideArm(-.45);

        sleep(1000);

        Arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void initVuforia() {

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 800;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
