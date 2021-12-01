package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bells {
    public static CRServo bellsServo;

    private static BELLS_STATE currentState = BELLS_STATE.OFF;

    private enum BELLS_STATE{
        FORWARD,
        OFF;
    }

    public Bells(){};

    public static void initBells(HardwareMap hwm){
        //Declare Motors on hardware map
        bellsServo = hwm.get(CRServo.class, "bellsServo");
        //bellsServo.setPower(0);
    }

    public static void spinBells(double power){
        bellsServo.setPower(power);
    }

    public static void stopBells(){
        bellsServo.setPower(0);
    }

    public static double getPower(){return bellsServo.getPower();}

    //Updates the powers for the bells based on the states above
    public static void bellsUpdatePosition() throws InterruptedException{
        if(currentState == BELLS_STATE.OFF){
            stopBells();
        }
        else if (currentState == BELLS_STATE.FORWARD){
            spinBells(.9);
        }
    }
    //States for the intake (Forward, backwards, stop)
    public static void bellsChangeState(){
        if(currentState == BELLS_STATE.OFF){
            currentState = BELLS_STATE.FORWARD;
        }
        else if (currentState == BELLS_STATE.FORWARD){
            currentState = BELLS_STATE.OFF;
        }
    }
}
