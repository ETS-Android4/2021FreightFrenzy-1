package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Auto {
    public static DcMotor deadWheel;

    public static final double timerLimit = 29;

    //Constructor
    public Auto(){}

    public static void initAuto(HardwareMap hwm){
        deadWheel = hwm.dcMotor.get("carouselMotor");

        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        deadWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public static void goToPosition(double targetYPos, double power, double allowedError, Telemetry telemetry, boolean opMode) throws InterruptedException{
        double distance = Math.abs(targetYPos - getYPositon());
        double initialDistance = Math.abs(targetYPos - getYPositon());
        double yMultiplier = 1.08;
        double previousDistance = 0;

        telemetry.addData("Distance To Y", distance);
        telemetry.update();

        while(opMode && (distance > allowedError)){
            /*if(runtime > timerLimit){
                DriveTrain.leftFront.setPower(0);
                DriveTrain.leftBack.setPower(0);
                DriveTrain.rightFront.setPower(0);
                DriveTrain.rightBack.setPower(0);
                Intake.stop();
                break;
            }
            */
            distance = Math.abs(targetYPos - getYPositon());

            double robotMovementYComponent = calculateY(0, power);

            if(distance/initialDistance < .25){
                yMultiplier += 0.1;
            }

            DriveTrain.leftFront.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward
            DriveTrain.rightFront.setPower(robotMovementYComponent * (distance/initialDistance) * yMultiplier);// + feedForward
            DriveTrain.leftBack.setPower  (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward
            DriveTrain.rightBack.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward

            previousDistance = distance;
        }

        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
        Thread.sleep(250);
    }

    public static int getYPositon(){
        return deadWheel.getCurrentPosition();
    }

    public void setZeroPower(int sleep) throws InterruptedException{
        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
        Thread.sleep(sleep);
    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private static double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }

    public static void autoBrake(int timer){
        int timerLength = timer;
        DriveTrain.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveTrain.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveTrain.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveTrain.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(timerLength > 0) {
            DriveTrain.leftFront.setTargetPosition(0);
            DriveTrain.leftBack.setTargetPosition(0);
            DriveTrain.rightFront.setTargetPosition(0);
            DriveTrain.rightBack.setTargetPosition(0);
            DriveTrain.setRunMode("RUN_TO_POSITION");
            DriveTrain.leftFront.setPower(0.8);
            DriveTrain.leftBack.setPower(0.8);
            DriveTrain.rightFront.setPower(0.8);
            DriveTrain.rightBack.setPower(0.8);
            timerLength--;
        }
        DriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
    }

    public static void resetEncoder(){
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void driveIntakeColor(double x, double y, int timer, Telemetry telemetry) {
        y = -y;
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = 0;
        double startingHeading = DriveTrain.angles.firstAngle;
        double currentError = 0;
        double adjustedXHeading = 0;
        double adjustedYHeading = 0;

        double currentColor;
        double exitValue;
        double currentDistance;
        double exitValueDistance;
        int timerLength = timer;

        currentColor = Double.MIN_VALUE;
        exitValue = Double.MAX_VALUE;

        currentDistance = Double.MAX_VALUE;
        exitValueDistance = Double.MIN_VALUE;


        while ((currentColor < exitValue && currentDistance > exitValueDistance) && timerLength >= 0) {
            exitValue = 1650;
            exitValueDistance = 10;

            DriveTrain.angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            adjustedXHeading = Math.cos(command + DriveTrain.angles.firstAngle + Math.PI / 4);
            adjustedYHeading = Math.sin(command + DriveTrain.angles.firstAngle + Math.PI / 4);

            currentError = DriveTrain.angles.firstAngle - startingHeading;

            if (Math.abs(currentError) > (Math.PI / 12)) {
                rotation = 0.40;
            } else {
                if (Math.abs(currentError) > (Math.PI / 180)) {
                    rotation = Math.abs(currentError / 0.6);
                } else {
                    rotation = 0;
                }
            }

            if (currentError < 0) {
                rotation = rotation * -1;
            }

            DriveTrain.leftFront.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
            DriveTrain.rightFront.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
            DriveTrain.leftBack.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
            DriveTrain.rightBack.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);

            currentColor = Intake.intakeFrontSensor.red();//leftDistanceSensor
            currentDistance = Arm.getArmSensorLength();

            telemetry.addData("Distance: ", Arm.armSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            timerLength--;
        }

        if(currentColor > exitValue)
            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);//AQUA
        if(currentDistance < exitValueDistance)
            DriveTrain.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);//AQUA

        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
    }
}
