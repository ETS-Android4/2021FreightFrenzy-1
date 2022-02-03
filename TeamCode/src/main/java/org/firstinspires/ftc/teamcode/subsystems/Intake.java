package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    //Declare motors
    public static DcMotor intakeFront;
    public static DcMotor intakeBack;

    //Declare Sensors
    public static ColorSensor intakeFrontSensor;
    public static ColorSensor intakeBackSensor;

    //Intake constants
    private static final double INTAKE_POWER = 0.95;

    //Constants for sensors
    private static final double FRONT_SENSOR = 1850;
    private static final double BACK_SENSOR = 1900;

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

        intakeFrontSensor = hwm.get(ColorSensor.class, "intakeFrontSensor");
        intakeBackSensor = hwm.get(ColorSensor.class, "intakeBackSensor");

        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //Intake forwards
    public static void intake() throws InterruptedException {
        intakeFront.setPower(INTAKE_POWER);
        intakeBack.setPower(INTAKE_POWER);
    }

    //Intake backwards
    public static void setBackwards(){
        intakeFront.setPower(-INTAKE_POWER);
        intakeBack.setPower(-INTAKE_POWER);
    }

    //Stop the intake
    public static void stop(){
        intakeFront.setPower(0);
        intakeBack.setPower(0);
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
