package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    //Declare motors
    public static DcMotor intake1;
    public static DcMotor intake2;

    //Intake constants
    private static final double INTAKE_POWER = 0.95;

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
        intake1 = hwm.get(DcMotor.class, "intake1");
        intake2 = hwm.get(DcMotor.class, "intake2");
    }

    //Intake forwards
    public static void intake() throws InterruptedException {
        intake1.setPower(INTAKE_POWER);
        intake2.setPower(INTAKE_POWER);
    }

    //Intake backwards
    public static void setBackwards(){
        intake1.setPower(-INTAKE_POWER);
        intake2.setPower(-INTAKE_POWER);
    }

    //Stop the intake
    public static void stop(){
        intake1.setPower(0);
        intake2.setPower(0);
    }

    //Changes the state of the intake based on the input
    public static void intakeChangeState(String direction){
        if(currentState == INTAKE_STATE.OFF && direction.equals("FORWARD")){
            currentState = INTAKE_STATE.INTAKE;
        }
        else if (currentState == INTAKE_STATE.INTAKE){
            currentState = INTAKE_STATE.OFF;
        }
        else if(currentState == INTAKE_STATE.OFF && direction.equals("REVERSE")){
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
