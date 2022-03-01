package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    //Declare motors
    public static DcMotor intakeFront;
    public static DcMotor intakeBack;

    //Declare Sensors
    public static ColorSensor intakeFrontSensor;
    public static ColorSensor intakeBackSensor;

    //Declare servos
    public static Servo frontServo;
    public static Servo backServo;

    //Intake constants
    private static final double INTAKE_FORWARD = 0.95;
    private static final double INTAKE_BACKWARDS = .65;

    //Constants for sensors
    private static double FRONT_SENSOR = 1950;
    private static double BACK_SENSOR = 1575;

    //Servo Constants
    private static final double FRONT_UP = .9;
    private static final double FRONT_DOWN = .0;
    private static final double BACK_UP = .1;
    private static final double BACK_DOWN = .83;

    //Intake State
    private static INTAKE_STATE currentState = INTAKE_STATE.OFF;

    private enum INTAKE_STATE{
        INTAKE,
        OFF,
        BACKWARDS;
    }

    //Constructor
    public Intake(){}

    public static void initIntake(HardwareMap hwm){
        //Declare Motors on hardware map
        intakeFront = hwm.get(DcMotor.class, "intakeFront");
        intakeBack = hwm.get(DcMotor.class, "intakeBack");

        frontServo = hwm.get(Servo.class, "frontServo");
        backServo = hwm.get(Servo.class, "backServo");

        intakeFrontSensor = hwm.get(ColorSensor.class, "intakeFrontSensor");
        intakeBackSensor = hwm.get(ColorSensor.class, "intakeBackSensor");

        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);

        frontIntakeUp();
        backIntakeUp();

        Intake.setFrontConstant();
        Intake.setBackConstant();
    }

    //Intake forwards
    public static void intake() throws InterruptedException {
        if(Intake.frontServo.getPosition() < .5)
            intakeFront.setPower(INTAKE_FORWARD);
        if(Intake.backServo.getPosition()  > .5)
            intakeBack.setPower(INTAKE_FORWARD);
    }

    //Intake backwards
    public static void setBackwards(){
        intakeFront.setPower(-INTAKE_BACKWARDS);
        intakeBack.setPower(-INTAKE_BACKWARDS);
    }

    //Stop the intake
    public static void stop(){
        intakeFront.setPower(0);
        intakeBack.setPower(0);
    }

    public static void setFrontConstant(){
        int sum = 0;
        for(int i = 0; i < 10; i++){
            sum += intakeFrontSensor.red();
        }
        FRONT_SENSOR = (sum / 10) + 50;
    }
    public static void setBackConstant(){
        int sum = 0;
        for(int i = 0; i < 10; i++){
            sum += intakeBackSensor.red();
        }
        BACK_SENSOR = (sum / 10) + 50;
    }

    public static void changeIntakeBackwards(){
        currentState = INTAKE_STATE.BACKWARDS;
    }

    public static void changeIntakeOff(){
        currentState = INTAKE_STATE.OFF;
    }

    public static boolean ballInFrontSensor(){
        if(intakeFrontSensor.red() > FRONT_SENSOR)
            return true;
        else
            return false;
    }

    public static boolean ballInBackSensor(){
        if(intakeBackSensor.red() > BACK_SENSOR)
            return true;
        else
            return false;
    }

    public static double getFrontConstant(){return FRONT_SENSOR;}

    public static double getBackConstant(){return BACK_SENSOR;}

    public static void frontIntakeUp(){
        frontServo.setPosition(FRONT_UP);
    }

    public static void frontIntakeDown(){
        frontServo.setPosition(FRONT_DOWN);
    }

    public static void backIntakeUp(){
        backServo.setPosition(BACK_UP);
    }

    public static void backIntakeDown(){
        backServo.setPosition(BACK_DOWN);
    }

    //Changes the state of the intake based on the input
    public static void intakeChangeState(String direction){
        if(currentState == INTAKE_STATE.OFF && direction.equals("FORWARD") && Arm.armIsIn()){
            currentState = INTAKE_STATE.INTAKE;
        }
        else if (currentState == INTAKE_STATE.INTAKE){
            currentState = INTAKE_STATE.OFF;
        }
        else if(currentState == INTAKE_STATE.OFF && direction.equals("REVERSE") && Arm.armIsIn()){
            currentState = INTAKE_STATE.BACKWARDS;
        }
        else if(currentState == INTAKE_STATE.BACKWARDS){
            currentState = INTAKE_STATE.OFF;
        }
    }

    //Updates the powers for the intake based on the states above
    public static void intakeUpdatePosition() throws InterruptedException{
        if(currentState == INTAKE_STATE.OFF){
            stop();
        }
        else if (currentState == INTAKE_STATE.INTAKE){
            intake();
        }
        else if(currentState == INTAKE_STATE.BACKWARDS){
            setBackwards();
        }
    }
}
