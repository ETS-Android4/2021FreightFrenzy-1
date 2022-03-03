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

@Autonomous(name= "BlueDuckCycle", group= "Autonomous")

public class BlueDuckCycle extends LinearOpMode {
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
                if (recognition.getLeft() < 215) {
                    label = "LEFT";
                } else if (recognition.getLeft() >= 215 && recognition.getLeft() <= 306) {
                    label = "MIDDLE";
                } else if (recognition.getLeft() > 306) {
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

        DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");

        Auto.powerMotorsIndiv(-.3, -.3, -.3, -.3);

        sleep(1000);

        Carousel.spin(.18);

        sleep(3500);

        Carousel.brake();

        Carousel.stop();

        DriveTrain.setRunMode("RUN_USING_ENCODER");

        Auto.powerMotors(0);

        //DriveTrain.cartesianDriveTimer(-.4, 0, 25);

        Auto.resetEncoder();

        Auto.goToPosition(3 * Constants.COUNTS_PER_INCH, .2, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        DriveTrain.gyroTurn(5 * Math.PI / 6, 500);
        //DriveTrain.customDrive(-.4, -.4, .4, .4, 15000);

        DriveTrain.resetGyro();

        Auto.resetEncoder();

        Auto.goToPosition(-2 * Constants.COUNTS_PER_INCH, -.2, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        Auto.resetEncoder();

        DriveTrain.cartesianDriveTimer(.8, 0, 30);

        if(Auto.getYPositon() < 0)
            Auto.goToPosition(0 * Constants.COUNTS_PER_INCH, .2, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        Auto.driveWallColorCarousel(-.05, telemetry);

        Auto.resetEncoder();

        DriveTrain.setRunMode("RUN_USING_ENCODER");

        //sleep(1000);

        //Auto.goToPosition(2 * Constants.COUNTS_PER_INCH, .1, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());

        if(label == null || label.equals("RIGHT")) {
            Arm.armFar();

            sleep(750);

            Arm.armOutUpLong();

            Arm.releaseFreight();

            Arm.arm.setPower(0);

            Arm.armInNoReset();
        } else if(label.equals("LEFT")){
            Arm.armDown();

            sleep(100);

            Arm.armOutLong();

            Arm.releaseFreight();

            Arm.arm.setPower(0);

            Arm.armInNoReset();
        }
        else if(label.equals("MIDDLE")){
            Arm.armMid();

            sleep(250);

            Arm.armOutLong();

            Arm.releaseFreight();

            Arm.arm.setPower(0);

            Arm.armInNoReset();
        }
        DriveTrain.setRunMode("RUN_USING_ENCODER");

        Auto.resetEncoder();

        Auto.goToPosition(7 * Constants.COUNTS_PER_INCH, .15, Constants.COUNTS_PER_INCH, telemetry, opModeIsActive());
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
