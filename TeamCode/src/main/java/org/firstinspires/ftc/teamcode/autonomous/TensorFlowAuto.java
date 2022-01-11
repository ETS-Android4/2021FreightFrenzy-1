package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

import java.util.List;

@Autonomous(name= "Tflow", group= "Autonomous")

public class TensorFlowAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
    private static final String VUFORIA_KEY = "AW/D0F3/////AAABmT6CO76ZukEWtNAvh1kty819QDEF9SG9ZxbfUcbjoxBCe0UcoTGK19TZdmHtWDwxhrL4idOt1tdJE+h9YGDtZ7U/njHEqSZ7jflzurT4j/WXTCjeTCSf0oMqcgduLTDNz+XEXMbPSlnHbO9ZnEZBun7HHr6N06kpYu6QZmG6WRvibuKCe5IeZJ21RoIeCsdp3ho/f/+QQLlnqaa1dw6i4xMFM0e2IaxujhQiWnd4by23CkMPvzKhy6YP3wPBq+awpzEPLDZcD8l1i0SqmX7HNpmw4kXBrWzEimAzp1aqONVau4kIwCGwJFusMdErw9IL7KQ5VqMKN4Xl67s0pwotoXsA+5SlWQAIodipYKZnPzwO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public String label;
    public boolean marker;

    private static List<Recognition> tfodRecogntions;

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

        /*for (Recognition recognition : tfodRecogntions) {
            if(recognition.getLeft() > 255){
                label = "RIGHT";
            }
            else if(recognition.getLeft() <= 255){
                label = "MIDDLE";
            }
        }

        if(tfodRecogntions.isEmpty()){
            label = "LEFT";
        }
         */

        for(Recognition recognition : tfodRecogntions) {
            telemetry.addData("Label: ", recognition.getLabel());
            if(recognition.getLabel().equals("Marker")) {
                marker = true;
            }else{
                marker = false;
            }
            if(!marker) {
                if (recognition.getLeft() < 135) {
                    label = "Left";
                } else if (recognition.getLeft() >= 135 && recognition.getLeft() <= 386) {
                    label = "Middle";
                } else if (recognition.getLeft() > 386) {
                    label = "Right";
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
        }

        waitForStart();

        if(label == null || label.equals("LEFT")){
            Arm.armDown();

            //Auto.goToPosition(-12 * Constants.COUNTS_PER_INCH, -.2, 250, telemetry, opModeIsActive());

            //Auto.autoBrake(25);

            //sleep(100);

            Arm.armOutDown();

            sleep(200);

            Arm.releaseFreight();

            sleep(500);

            Arm.armIn();

            //finishes preloaded block, begin first cycle

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, .25, 15);

            Auto.driveIntakeColor(0, .12, 35, telemetry);

            sleep(50);

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                Auto.driveIntakeColor(0, .12, 35, telemetry);
            }
            else{
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            DriveTrain.cartesianDriveTimer(0, -.25, 10);

            if(Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            Intake.setBackwards();

            DriveTrain.cartesianDriveTimer(.4, -.1, 25);

            Arm.armMid();

            DriveTrain.driveToWhiteLineBlue(-.2, "WHITE", telemetry);

            //DriveTrain.cartesianDriveTimer(.4, 0, 13);

            Auto.resetEncoder();

            Intake.stop();

            Auto.goToPosition(-35 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
            sleep(100);

            Auto.autoBrake(25);

            Arm.armOutMid();

            sleep(100);

            Arm.releaseFreight();

            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            sleep(100);

            Arm.armIn();

            Arm.armDown();

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, .25, 20);

            Auto.driveIntakeColor(0, .12, 35, telemetry);

            sleep(50);

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                Auto.driveIntakeColor(0, .12, 35, telemetry);
            }
            else{
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            DriveTrain.cartesianDriveTimer(0, -.25, 10);

            if(Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            Intake.setBackwards();

            DriveTrain.cartesianDriveTimer(.4, -.1, 25);

            Arm.armMid();

            DriveTrain.driveToWhiteLineBlue(-.2, "WHITE", telemetry);

            //DriveTrain.cartesianDriveTimer(.4, 0, 13);

            Auto.resetEncoder();

            Intake.stop();

            Auto.goToPosition(-35 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
            sleep(100);

            Auto.autoBrake(50);

            Arm.armOutMid();

            sleep(100);

            Arm.releaseFreight();

            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            sleep(100);

            Arm.armIn();

            Arm.armDown();

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            DriveTrain.cartesianDriveTimer(0, .25, 15);

            DriveTrain.cartesianDriveTimer(-.4, 0, 60);
        }
        else if(label.equals("MIDDLE")){
            Arm.armMid();

            //DriveTrain.cartesianDriveTimer(0, -.4, 20);

            //sleep(100);

            Arm.armOutMid();

            sleep(200);

            Arm.releaseFreight();

            Arm.armDown();

            Arm.armIn();

            //finishes preloaded block, begin first cycle

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, .25, 15);

            Auto.driveIntakeColor(0, .12, 35, telemetry);

            sleep(50);

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                Auto.driveIntakeColor(0, .12, 35, telemetry);
            }
            else{
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            DriveTrain.cartesianDriveTimer(0, -.25, 10);

            if(Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            Intake.setBackwards();

            DriveTrain.cartesianDriveTimer(.4, -.1, 25);

            Arm.armMid();

            DriveTrain.driveToWhiteLineBlue(-.2, "WHITE", telemetry);

            //DriveTrain.cartesianDriveTimer(.4, 0, 13);

            Auto.resetEncoder();

            Intake.stop();

            Auto.goToPosition(-35 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
            sleep(100);

            Auto.autoBrake(25);

            Arm.armOutMid();

            sleep(100);

            Arm.releaseFreight();

            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            sleep(100);

            Arm.armIn();

            Arm.armDown();

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, .25, 20);

            Auto.driveIntakeColor(0, .12, 35, telemetry);

            sleep(50);

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                Auto.driveIntakeColor(0, .12, 35, telemetry);
            }
            else{
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            DriveTrain.cartesianDriveTimer(0, -.25, 10);

            if(Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            Intake.setBackwards();

            DriveTrain.cartesianDriveTimer(.4, -.1, 25);

            Arm.armMid();

            DriveTrain.driveToWhiteLineBlue(-.2, "WHITE", telemetry);

            //DriveTrain.cartesianDriveTimer(.4, 0, 13);

            Auto.resetEncoder();

            Intake.stop();

            Auto.goToPosition(-35 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
            sleep(100);

            Auto.autoBrake(50);

            Arm.armOutMid();

            sleep(100);

            Arm.releaseFreight();

            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            sleep(100);

            Arm.armIn();

            Arm.armDown();

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            DriveTrain.cartesianDriveTimer(0, .25, 15);

            DriveTrain.cartesianDriveTimer(-.4, 0, 60);
        }
        else if(label.equals("RIGHT")){
            Arm.armUp();

            sleep(250);

            //DriveTrain.cartesianDriveTimer(0, -.4, 20);

            //sleep(100);

            Arm.armOutUp();

            sleep(200);

            Arm.releaseFreight();

            sleep(500);

            Arm.armDown();

            Arm.armIn();

            //finishes preloaded block, begin first cycle

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, .25, 15);

            Auto.driveIntakeColor(0, .12, 35, telemetry);

            sleep(50);

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                Auto.driveIntakeColor(0, .12, 35, telemetry);
            }
            else{
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            DriveTrain.cartesianDriveTimer(0, -.25, 10);

            if(Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            Intake.setBackwards();

            DriveTrain.cartesianDriveTimer(.4, -.1, 25);

            Arm.armMid();

            DriveTrain.driveToWhiteLineBlue(-.2, "WHITE", telemetry);

            //DriveTrain.cartesianDriveTimer(.4, 0, 13);

            Auto.resetEncoder();

            Intake.stop();

            Auto.goToPosition(-35 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
            sleep(100);

            Auto.autoBrake(25);

            Arm.armOutMid();

            sleep(100);

            Arm.releaseFreight();

            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            sleep(100);

            Arm.armDown();

            Arm.armIn();

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, .25, 20);

            Auto.driveIntakeColor(0, .12, 35, telemetry);

            sleep(50);

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            if(Intake.intakeFrontSensor.red() < 1650 && Arm.getArmSensorLength() > 10){
                Auto.driveIntakeColor(0, .12, 35, telemetry);
            }
            else{
                DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            DriveTrain.cartesianDriveTimer(0, -.25, 10);

            if(Arm.getArmSensorLength() > 10){
                int timerLength = 35;
                while(timerLength > 0 && Arm.getArmSensorLength() > 10){
                    timerLength--;
                }
            }

            Intake.setBackwards();

            DriveTrain.cartesianDriveTimer(.4, -.1, 25);

            Arm.armMid();

            DriveTrain.driveToWhiteLineBlue(-.2, "WHITE", telemetry);

            //DriveTrain.cartesianDriveTimer(.4, 0, 13);

            Auto.resetEncoder();

            Intake.stop();

            Auto.goToPosition(-35 * Constants.COUNTS_PER_INCH, -.25, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
            sleep(100);

            Auto.autoBrake(50);

            Arm.armOutMid();

            sleep(100);

            Arm.releaseFreight();

            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            sleep(100);

            Arm.armDown();

            Arm.armIn();

            DriveTrain.driveToWhiteLineBlue(.22,"WHITE", telemetry);

            DriveTrain.cartesianDriveTimer(0, .25, 15);

            DriveTrain.cartesianDriveTimer(-.4, 0, 60);
        }

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
        tfodParameters.minResultConfidence = 0.3f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 800;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
