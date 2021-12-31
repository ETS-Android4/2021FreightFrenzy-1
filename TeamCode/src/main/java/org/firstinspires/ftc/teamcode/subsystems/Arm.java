package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm {
    //Motors
    public static DcMotor arm;

    //Servos
    public static Servo heightServo1;
    public static Servo heightServo2;
    public static Servo vibrator;

    //Sensors
    public static DistanceSensor armSensor;
    public static DistanceSensor gondolaSensor;

    //Constants for flicker servo
    private static final double VIBRATOR_CLOSED = 0.14;
    private static final double VIBRATOR_OPEN = 0.4;

    //Constants for height servos
    private static final double ARM_DOWN = 0.65;
    private static final double ARM_UP = 0.35;
    private static final double ARM_MID = 0.5;
    private static final double ARM_MAX = 0.2;

    //Constants for arm powers
    private static final double ARM_FAST = .6;
    private static final double ARM_SLOW = .2;
    private static String speed = "FAST";

    //States
    private static ARM_STATE currentArmState = ARM_STATE.IN;
    private static HEIGHT_STATE currentHeightState = HEIGHT_STATE.DOWN;

    private enum ARM_STATE{
        OUT_FAST,
        OUT_SLOW,
        IN_FAST,
        IN_SLOW,
        OUT_UP,
        OUT,
        IN;
    }

    private enum HEIGHT_STATE{
        UP,
        DOWN,
        MID,
        MAX;
    }

    //Constructor
    public Arm(){}

    public static void initArm(HardwareMap hwm){
        //Declare Motors on hardware map
        arm = hwm.get(DcMotor.class, "arm");

        heightServo1 = hwm.get(Servo.class, "heightServo1");
        heightServo2 = hwm.get(Servo.class, "heightServo2");
        vibrator = hwm.get(Servo.class, "vibrator");

        armSensor = hwm.get(DistanceSensor.class, "armSensor");
        gondolaSensor = hwm.get(DistanceSensor.class, "gondolaSensor");

        //Init Servos
        heightServo1.setPosition(ARM_DOWN);
        heightServo2.setPosition(ARM_DOWN);
        vibrator.setPosition(VIBRATOR_CLOSED);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void moveArm(double newHeight){
        heightServo1.setPosition(newHeight);
        heightServo2.setPosition(newHeight);
    }

    public static void armUp(){
        heightServo1.setPosition(ARM_UP);
        heightServo2.setPosition(ARM_UP);
    }

    public static void armDown(){
        heightServo1.setPosition(ARM_DOWN);
        heightServo2.setPosition(ARM_DOWN);
    }

    public static void armMid(){
        heightServo1.setPosition(ARM_MID);
        heightServo2.setPosition(ARM_MID);
    }

    public static void armMax(){
        heightServo1.setPosition(ARM_MAX);
        heightServo2.setPosition(ARM_MAX);
    }

    public static void releaseFreight() throws InterruptedException {
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(200);
        vibrator.setPosition(VIBRATOR_CLOSED);
    }

    public static void slideArm(double power){
        arm.setPower(power);
    }

    public static void stopArm(){
        arm.setPower(0);
    }

    public static void armOutUp(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(arm.getCurrentPosition() < 1000){
            arm.setTargetPosition(1000);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(ARM_FAST);
        }
        arm.setPower(0);
        currentArmState = ARM_STATE.OUT;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void armOutMid(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(arm.getCurrentPosition() < 950){
            arm.setTargetPosition(950);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(ARM_FAST);
        }
        arm.setPower(0);
        currentArmState = ARM_STATE.OUT;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void armOutDown(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(arm.getCurrentPosition() < 900){
            arm.setTargetPosition(900);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(ARM_FAST);
        }
        arm.setPower(0);
        currentArmState = ARM_STATE.OUT;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void armIn(){
        while(arm.getCurrentPosition() > 0){
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(-ARM_FAST);
        }
        arm.setPower(0);
        currentArmState = ARM_STATE.IN;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static ARM_STATE getArmState(){
        return currentArmState;
    }

    public static boolean armIsIn(){
        if(currentArmState == ARM_STATE.IN)
            return true;
        else
            return false;
    }

    public static void changeSpeed(String newSpeed){
        speed = newSpeed;
    }

    public static String getSpeed(){
        return speed;
    }

    public static double getArmSensorLength(){
        return armSensor.getDistance(DistanceUnit.CM);
    }

    //Changes the state of the height servos based on the input
    public static void heightChangeState(String pos){
        if(pos.equals("UP")){
            currentHeightState = HEIGHT_STATE.UP;
        }
        else if(pos.equals("DOWN")){
            currentHeightState = HEIGHT_STATE.DOWN;
        }
        else if(pos.equals("MID")){
            currentHeightState = HEIGHT_STATE.MID;
        }
        else if(pos.equals("MAX")){
            currentHeightState = HEIGHT_STATE.MAX;
        }
    }

    public static void heightUpdatePosition() {
        if(currentHeightState == HEIGHT_STATE.DOWN){
            armDown();
        }
        else if (currentHeightState == HEIGHT_STATE.UP){
            armUp();
        }
        else if (currentHeightState == HEIGHT_STATE.MID){
            armMid();
        }
        else if (currentHeightState == HEIGHT_STATE.MAX){
            armMax();
        }
    }

    public static void armChangeState(String dir){
        if(speed.equals("FAST") && dir.equals("OUT")){
            currentArmState = ARM_STATE.OUT_FAST;
        }
        else if(speed.equals("SLOW") && dir.equals("OUT")){
            currentArmState = ARM_STATE.OUT_SLOW;
        }
        else if(speed.equals("FAST") && dir.equals("IN")){
            currentArmState = ARM_STATE.IN_FAST;
        }
        else if(speed.equals("SLOW") && dir.equals("IN")){
            currentArmState = ARM_STATE.IN_SLOW;
        }
        else if(currentArmState == ARM_STATE.OUT_SLOW || currentArmState == ARM_STATE.OUT_FAST){
            currentArmState = ARM_STATE.OUT;
        }
        else if(currentArmState == ARM_STATE.IN_SLOW || currentArmState == ARM_STATE.IN_FAST){
            if(arm.getCurrentPosition() < 100){
                stopArm();
                currentArmState = ARM_STATE.IN;
            }
            else{
                currentArmState = ARM_STATE.OUT;
            }
        }
        /* if(currentArmState == ARM_STATE.IN){
            currentArmState = ARM_STATE.OUT_UP;
        }*/
    }

    public static void armUpdatePosition(){
        if(currentArmState == ARM_STATE.IN){
            //arm.setPower(-.2);
            stopArm();
        }
        else if(currentArmState == ARM_STATE.OUT){
            stopArm();
        }
        else if(currentArmState == ARM_STATE.OUT_SLOW){
            slideArm(-.2);
        }
        else if(currentArmState == ARM_STATE.OUT_FAST){
            slideArm(.6);
        }
        else if(currentArmState == ARM_STATE.IN_SLOW){
            slideArm(-.2);
        }
        else if(currentArmState == ARM_STATE.IN_FAST){
            slideArm(-.6);
        }
        /*else if(currentArmState == ARM_STATE.OUT_UP){
            armOutUp();
        }*/
    }
}
