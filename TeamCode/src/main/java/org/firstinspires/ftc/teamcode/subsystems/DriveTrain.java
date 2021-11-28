package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
