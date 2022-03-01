package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name= "RedTwoCycle", group= "Autonomous")
@Disabled

public class RedTwoCycle extends LinearOpMode {
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

        if(label == null || label.equals("RIGHT")) {
            Arm.armUp();

            sleep(900);

            Arm.armOutUpSlow();

            sleep(200);

            Arm.releaseFreight();

            sleep(100);

            Arm.armDown();

            Arm.armInNoReset();
        }else if(label.equals("LEFT")){
            Arm.armDown();

            Arm.armOutDown();

            sleep(200);

            Arm.releaseFreight();

            sleep(100);

            Arm.armInNoReset();
        }
        else if(label.equals("MIDDLE")){
            Arm.armMid();

            sleep(250);

            Arm.armOutMid();

            Arm.releaseFreight();

            sleep(100);

            Arm.armDown();

            Arm.armInNoReset();
        }


        Auto.goToPosition(-43 * Constants.COUNTS_PER_INCH, -.35, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        Arm.armIn();

        Intake.intake();

        while(!Intake.ballInFrontSensor() && !Arm.ballInGondola()){
            Auto.driveIntakeColor(-.15, 20, telemetry);

            telemetry.addData("First", "");

            sleep(50);

            if(!Intake.ballInFrontSensor() && !Arm.ballInGondola()){
                DriveTrain.cartesianDriveTimer(0, .2, 25);
                telemetry.addData("Second", "");
            }
            else{
                break;
            }
        }

        if(Arm.ballInGondola()){
            Intake.setBackwards();
        }

        DriveTrain.driveToLineBlue(.2, "WHITE", telemetry);

        Arm.armUp();

        Intake.setBackwards();

        Auto.resetEncoder();

        Auto.goToPosition(39 * Constants.COUNTS_PER_INCH, .35, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
        sleep(100);

        //Auto.autoBrake(25);

        Arm.armOutUp();

        sleep(100);

        Arm.releaseFreight();

        sleep(100);

        Arm.armDown();

        Arm.armInNoReset();

        Intake.stop();

        Auto.resetEncoder();

        DriveTrain.cartesianDriveTimer(.8, 0, 10);

        Auto.goToPosition(-41 * Constants.COUNTS_PER_INCH, -.35, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        Arm.armIn();

        Intake.intake();

        while(!Intake.ballInFrontSensor() && !Arm.ballInGondola()){
            Auto.driveIntakeColor(-.15, 20, telemetry);

            telemetry.addData("First", "");

            sleep(50);

            if(!Intake.ballInFrontSensor() && !Arm.ballInGondola()){
                DriveTrain.cartesianDriveTimer(0, .2, 25);
                telemetry.addData("Second", "");
            }
            else{
                break;
            }
        }

        if(Arm.ballInGondola()){
            Intake.setBackwards();
        }

        DriveTrain.driveToLineBlue(.2, "WHITE", telemetry);

        Arm.armUp();

        Intake.setBackwards();

        Auto.resetEncoder();

        Auto.goToPosition(39 * Constants.COUNTS_PER_INCH, .35, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
        sleep(100);

        //Auto.autoBrake(25);

        Arm.armOutUp();

        sleep(100);

        Arm.releaseFreight();

        sleep(100);

        Arm.armDown();

        Arm.armInNoReset();

        Intake.stop();

        Auto.resetEncoder();

        Auto.goToPosition(-50 * Constants.COUNTS_PER_INCH, -.35, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        DriveTrain.cartesianDriveTimer(-.4, 0, 30);
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
