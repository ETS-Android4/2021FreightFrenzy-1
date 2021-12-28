package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

public class DriveTrain {
    //Declare Motors
    public static DcMotor leftFront;
    public static DcMotor leftBack;
    public static DcMotor rightFront;
    public static DcMotor rightBack;

    public static BNO055IMU imu;
    public static Orientation angles;
    public static Acceleration gravity;

    //Sensors
    public static ColorSensor floorColorSensor;

    //Turning
    private static double driveTrainError = 0;
    private static double driveTrainPower = 0;

    //Constructor
    public DriveTrain(){}

    //Initialize
    public static void initDriveTrain(HardwareMap hwm){
        //Declare Motors on hardware map
        leftFront = hwm.get(DcMotor.class, "leftFront");
        leftBack = hwm.get(DcMotor.class, "leftBack");
        rightFront = hwm.get(DcMotor.class, "rightFront");
        rightBack = hwm.get(DcMotor.class, "rightBack");

        imu = hwm.get(BNO055IMU.class, "imu");

        floorColorSensor = hwm.get(ColorSensor.class, "floorColorSensor");

        //Reverse Motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public static void cartesianDrive(double x, double y, double z){
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = z;

        if(z > 0) {
            if (z < 0.01) {
                rotation = 0;
            } else if (z > 0.9) {
                rotation = z * z;
            } else {
                rotation = (z * z) + 0.013;
            }
        }
        if(z < 0) {
            if (z > -0.01) {
                rotation = 0;
            } else if (z < -0.9) {
                rotation = -(z * z);
            }
            else{
                rotation = (-(z*z)) - 0.013;
            }
        }

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI/4);
        double adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI/4);

        leftFront.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
        rightFront.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
        leftBack.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
        rightBack.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);

    }

    //This is our autonomous cartesian drive method. It utilizes the same math as the normal cartesian drive method
    public static void cartesianDriveTimer(double x, double y, int timerLength) throws InterruptedException {
        y = -y;
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = 0;
        double startingHeading = angles.firstAngle;
        double currentError = 0;
        double adjustedXHeading = 0;
        double adjustedYHeading = 0;


        while(timerLength > 0) {
            angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI / 4);
            adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI / 4);

            currentError = angles.firstAngle - startingHeading;

            if(Math.abs(currentError) > (Math.PI / 12)){
                rotation = 0.40;
            }
            else{
                if(Math.abs(currentError) > (Math.PI / 180)){
                    rotation = Math.abs(currentError / 0.6);
                }
                else{
                    rotation = 0;
                }
            }

            if(currentError < 0){
                rotation = rotation * -1;
            }

            leftFront.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
            rightFront.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
            leftBack.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
            rightBack.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);
            Thread.sleep(20);
            timerLength--;
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    //Turn using the gyro in auto
    public static void gyroTurn(double finalAngle, int timer){
        int turnTimer = timer;
        while (Math.abs(DriveTrain.angles.firstAngle - (finalAngle)) > (Math.PI / 60) && turnTimer > 0){
            driveTrainError = angles.firstAngle - finalAngle;
            if(Math.abs(driveTrainError) > (Math.PI / 6)){
                driveTrainPower = 0.5;
            }
            else{
                if(Math.abs(driveTrainError) < (Math.PI / 60)){//60
                    driveTrainPower = 0;
                }
                else if(Math.abs(driveTrainError) > (Math.PI / 60)) {//60
                    driveTrainPower = Math.abs(driveTrainError / (Math.PI / 2)) + 0.1;
                }
            }
            driveTrainError = angles.firstAngle - finalAngle;
            if(driveTrainError > 0){
                cartesianDrive(0, 0, driveTrainPower);
            }
            else if(driveTrainError < 0){
                cartesianDrive(0, 0, -driveTrainPower);
            }
            turnTimer--;
        }
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
    }

    public static void driveToLine(double power, String color, Telemetry telemetry) throws InterruptedException {
        double minBlue = Double.MAX_VALUE;
        double maxBlue = Double.MIN_VALUE;

        double minWhite = Double.MAX_VALUE;
        double maxWhite = Double.MIN_VALUE;
        if(color.equals("RED")){
            while(floorColorSensor.red() < 1350){ //1600
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
        else if(color.equals("BLUE")){
            while(floorColorSensor.blue() < 1600){//2100
                if(DriveTrain.floorColorSensor.blue() > maxBlue){
                    maxBlue = DriveTrain.floorColorSensor.blue();
                }

                if (DriveTrain.floorColorSensor.blue() < minBlue){
                    minBlue = DriveTrain.floorColorSensor.blue();
                }
                telemetry.addData("Max Blue: ", maxBlue);
                telemetry.addData("Min Blue: ", minBlue);
                telemetry.update();
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
        else if(color.equals("WHITE")) {
            while(floorColorSensor.alpha() < 120){//480, 680
                if(DriveTrain.floorColorSensor.alpha() > maxWhite){
                    maxWhite = DriveTrain.floorColorSensor.alpha();
                }

                if (DriveTrain.floorColorSensor.alpha() < minWhite){
                    minWhite = DriveTrain.floorColorSensor.alpha();
                }
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
//                Intake.releaseAll();
                telemetry.addData("Max White: ", maxWhite);
                telemetry.addData("Min White: ", minWhite);
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }

    //This is our favorite method by far. It sets the encoder positions to zero and stays there. This way, we can move at the end of auto.
    public static void SUMO_MODE(){
        DriveTrain.leftFront.setTargetPosition(0);
        DriveTrain.leftBack.setTargetPosition(0);
        DriveTrain.rightFront.setTargetPosition(0);
        DriveTrain.rightBack.setTargetPosition(0);
        DriveTrain.setRunMode("RUN_TO_POSITION");
        DriveTrain.leftFront.setPower(0.8);
        DriveTrain.leftBack.setPower(0.8);
        DriveTrain.rightFront.setPower(0.8);
        DriveTrain.rightBack.setPower(0.8);
    }

    public static void autoBrake(int timer){
        double currentPower = leftFront.getPower();
        double power = -currentPower * 2;

        while(timer > 0) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            timer--;
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public static void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public static void composeTelemetry (Telemetry telemetry) {

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                gravity = imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    public static void gyroTele(Telemetry telemetry){
        telemetry.addData("Heading: ", formatAngle(angles.angleUnit, angles.firstAngle));
    }
    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.RADIANS.fromUnit(angleUnit, angle));
    }
    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.RADIANS.normalize(degrees));
    }
    public static void resetGyro(){
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

}
