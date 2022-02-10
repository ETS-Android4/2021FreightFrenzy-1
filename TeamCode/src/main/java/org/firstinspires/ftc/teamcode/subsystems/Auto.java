package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

        DriveTrain.blinkinLedDriver.close();
    }

    /*public static void goToPosition(double targetYPos, double power, double allowedError, Telemetry telemetry, boolean opMode) throws InterruptedException{
        double distance = Math.abs(targetYPos - getYPositon());
        double initialDistance = Math.abs(targetYPos - getYPositon());
        double yMultiplier = 1.08;
        double previousAct;
        double currentAct;
        double previousY;
        double currentY;
        if(power >= 0) {
            previousAct = getLeftFront();
        }
        else {
            previousAct = getRightFront();
        }
        previousY = getYPositon();
        double rrMultipler = 2.25;

        telemetry.addData("Distance To Y", distance);
        telemetry.update();

        int i = 1;

        while(opMode && (distance > allowedError)){

            distance = Math.abs(targetYPos - getYPositon());

            double robotMovementYComponent = calculateY(0, power);

            if(distance/initialDistance < .25){
                yMultiplier += 0.1;
            }

            if(power >= 0) {
                if (i % 5 == 0) {
                    currentY = getYPositon();
                    currentAct = getLeftFront();
                    double actDif = Math.abs(currentAct - previousAct);
                    double yDif = Math.abs(currentY - previousY);
                    telemetry.addData("YDif: ", yDif);
                    telemetry.addData("ActDif: ", actDif);
                    telemetry.update();
                    if (yDif != 0 && actDif != 0) {
                        if (Math.abs(actDif / yDif) > 1) {
                            opMode = false;
                        }
                    }
                }
                DriveTrain.leftFront.setPower(robotMovementYComponent * (distance / initialDistance) * yMultiplier); //+ feedForward
                DriveTrain.rightFront.setPower(robotMovementYComponent * (distance / initialDistance) * yMultiplier * rrMultipler);// + feedForward
                DriveTrain.leftBack.setPower(robotMovementYComponent * (distance / initialDistance) * yMultiplier * rrMultipler); //+ feedForward
                DriveTrain.rightBack.setPower(robotMovementYComponent * (distance / initialDistance) * yMultiplier); //+ feedForward
                if ((i - 1) % 5 == 0){
                    previousAct = getLeftFront();
                    previousY = getYPositon();
                }
            }
            else if (power < 0){

                currentY = getRightFront();
                double actDif = Math.abs(currentY - previousY);
                double yDif = Math.abs(currentAct - previousAct);
                if(yDif != 0){
                    if(Math.abs(actDif / yDif) > 10){
                        opMode = false;
                    }
                }
                DriveTrain.leftFront.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier * rrMultipler); //+ feedForward
                DriveTrain.rightFront.setPower(robotMovementYComponent * (distance/initialDistance) * yMultiplier);// + feedForward
                DriveTrain.leftBack.setPower  (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward
                DriveTrain.rightBack.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier * rrMultipler); //+ feedForward
                previousY = getRightFront();


            }



            telemetry.addData("Distance To Y", distance);
            telemetry.addData("I: ", i);
            telemetry.addData("Left front power: ", DriveTrain.leftFront.getPower());
            i++;
        }

        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
        Thread.sleep(250);
    }
    */

    public static void goToPosition(double targetYPos, double power, double allowedError, Telemetry telemetry, boolean opMode) throws InterruptedException{
        double distance = Math.abs(targetYPos - getYPositon());
        double initialDistance = Math.abs(targetYPos - getYPositon());
        double yMultiplier = 1.08;
        double rrMultipler = 2.25;

        telemetry.addData("Distance To Y", distance);
        telemetry.update();

        while(opMode && (distance > allowedError)){
            distance = Math.abs(targetYPos - getYPositon());

            double robotMovementYComponent = calculateY(0, power);

            if(distance/initialDistance < .25){
                yMultiplier += 0.1;
            }

            if(power >= 0){
                DriveTrain.leftFront.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward
                DriveTrain.rightFront.setPower(robotMovementYComponent * (distance/initialDistance) * yMultiplier * rrMultipler);// + feedForward
                DriveTrain.leftBack.setPower  (robotMovementYComponent * (distance/initialDistance) * yMultiplier * rrMultipler); //+ feedForward
                DriveTrain.rightBack.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward
            }
            else if (power < 0){
                DriveTrain.leftFront.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier * rrMultipler); //+ feedForward
                DriveTrain.rightFront.setPower(robotMovementYComponent * (distance/initialDistance) * yMultiplier);// + feedForward
                DriveTrain.leftBack.setPower  (robotMovementYComponent * (distance/initialDistance) * yMultiplier); //+ feedForward
                DriveTrain.rightBack.setPower (robotMovementYComponent * (distance/initialDistance) * yMultiplier * rrMultipler); //+ feedForward
            }

            telemetry.addData("Distance To Y", distance);
            telemetry.addData("Left front power: ", DriveTrain.leftFront.getPower());
            telemetry.update();
        }

        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
    }

    public static int getYPositon(){
        return deadWheel.getCurrentPosition();
    }

    public static int getRightFront(){return DriveTrain.rightFront.getCurrentPosition();}

    public static int getLeftFront(){return DriveTrain.leftFront.getCurrentPosition();}

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
        double power = .8;
        double multiplier = 0.25;
        while(timerLength > 0) {
            DriveTrain.leftFront.setTargetPosition(0);
            DriveTrain.leftBack.setTargetPosition(0);
            DriveTrain.rightFront.setTargetPosition(0);
            DriveTrain.rightBack.setTargetPosition(0);
            DriveTrain.setRunMode("RUN_TO_POSITION");


            DriveTrain.leftFront.setPower(power);
            DriveTrain.rightFront.setPower(power * (1 + multiplier));
            DriveTrain.leftBack.setPower(power);
            DriveTrain.rightBack.setPower(power * (1 - multiplier));


            timerLength--;
        }
        /*
        DriveTrain.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);

         */
    }

    public static void resetEncoder(){
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void driveIntakeColor(double y, int timer, Telemetry telemetry) {
        double currentColorFront;
        double currentColorBack;
        double exitValueFront;
        double exitValueBack;
        double currentDistance;
        double exitValueDistance;
        int timerLength = timer;

        currentColorFront = Double.MIN_VALUE;
        currentColorBack = Double.MIN_VALUE;
        exitValueFront = Double.MAX_VALUE;
        exitValueBack = Double.MAX_VALUE;

        currentDistance = Double.MAX_VALUE;
        exitValueDistance = Double.MIN_VALUE;


        while ((currentColorFront < exitValueFront && currentColorBack < exitValueBack && currentDistance > exitValueDistance) && timerLength >= 0) {
            exitValueFront = Intake.getFrontConstant();
            exitValueBack = Intake.getBackConstant();
            exitValueDistance = Arm.getGondolaConstant();

            if(y < 0) {
                DriveTrain.leftFront.setPower(y * Constants.TELEOP_LIMITER * 2);
            }
            else{
                DriveTrain.leftFront.setPower(y * Constants.TELEOP_LIMITER);
            }
            if(y > 0) {
                DriveTrain.rightFront.setPower(y * Constants.TELEOP_LIMITER * 2);
            }
            else{
                DriveTrain.rightFront.setPower(y * Constants.TELEOP_LIMITER);
            }
            DriveTrain.leftBack.setPower(y * Constants.TELEOP_LIMITER);
            DriveTrain.rightBack.setPower(y * Constants.TELEOP_LIMITER);

            currentColorFront = Intake.intakeFrontSensor.red();
            currentColorBack = Intake.intakeBackSensor.red();
            currentDistance = Arm.getArmSensorLength();

            timerLength--;
        }

        DriveTrain.leftFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.rightBack.setPower(0);
    }

    public static void driveWallColor(double power, Telemetry telemetry) throws InterruptedException {
        double multiplier = 2.25;

        double minWhite = Double.MAX_VALUE;
        double maxWhite = Double.MIN_VALUE;
        double exitValue = DriveTrain.wallColorSensor.alpha() + 200;

        int i = 0;
        do{
            i++;
            if(i == 50)
                Intake.setBackwards();
            if(DriveTrain.wallColorSensor.alpha() > maxWhite){
                maxWhite = DriveTrain.wallColorSensor.alpha();
            }
            if (DriveTrain.wallColorSensor.alpha() < minWhite){
                minWhite = DriveTrain.wallColorSensor.alpha();
            }
            if(power >= 0){
                DriveTrain.leftFront.setPower(power);
                DriveTrain.rightFront.setPower(power * multiplier);
                DriveTrain.leftBack.setPower(power * multiplier);
                DriveTrain.rightBack.setPower(power);
            }
            else if (power < 0){
                DriveTrain.leftFront.setPower(power * multiplier);
                DriveTrain.rightFront.setPower(power);
                DriveTrain.leftBack.setPower(power);
                DriveTrain.rightBack.setPower(power * multiplier);
            }
            if(Arm.ballInGondola()){
                Intake.setBackwards();
            }
            telemetry.addData("Max White: ", maxWhite);
            telemetry.addData("Min White: ", minWhite);
            telemetry.update();
        }while(maxWhite < exitValue && DriveTrain.wallColorSensor.alpha() < exitValue);

        minWhite = Double.MAX_VALUE;
        maxWhite = Double.MIN_VALUE;

        Intake.setBackwards();

        do{
            if(Arm.ballInGondola())
                Arm.armOutUpFast();
            if(DriveTrain.wallColorSensor.alpha() > maxWhite){
                maxWhite = DriveTrain.wallColorSensor.alpha();
            }
            if (DriveTrain.wallColorSensor.alpha() < minWhite){
                minWhite = DriveTrain.wallColorSensor.alpha();
            }
            if(power >= 0){
                DriveTrain.leftFront.setPower(power);
                DriveTrain.rightFront.setPower(power * multiplier);
                DriveTrain.leftBack.setPower(power * multiplier);
                DriveTrain.rightBack.setPower(power);
            }
            else if (power < 0){
                DriveTrain.leftFront.setPower(power * multiplier);
                DriveTrain.rightFront.setPower(power);
                DriveTrain.leftBack.setPower(power);
                DriveTrain.rightBack.setPower(power * multiplier);
            }
            if(Arm.ballInGondola()){
                Intake.setBackwards();
            }
            telemetry.addData("Max White: ", maxWhite);
            telemetry.addData("Min White: ", minWhite);
            telemetry.update();
        }while(maxWhite > exitValue && DriveTrain.wallColorSensor.alpha() > exitValue);
        DriveTrain.leftFront.setPower(0);
        DriveTrain.rightFront.setPower(0);
        DriveTrain.leftBack.setPower(0);
        DriveTrain.rightBack.setPower(0);

    }

    public static void powerMotors(double power){
        DriveTrain.leftFront.setPower(power);
        DriveTrain.leftBack.setPower(power);
        DriveTrain.rightFront.setPower(power);
        DriveTrain.rightBack.setPower(power);
    }

    public static void powerMotorsIndiv(double lf, double lb, double rf, double rb){
        DriveTrain.leftFront.setPower(lf);
        DriveTrain.leftBack.setPower(lb);
        DriveTrain.rightFront.setPower(rf);
        DriveTrain.rightBack.setPower(rb);
    }
}
