package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

            /*if(Math.abs(distance - previousDistance) < 15){
                yMultiplier += 0.04;
            }*/

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
}
