package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    //Motors
    public static DcMotor arm;

    //Servos
    public static Servo heightServo1;
    public static Servo heightServo2;
    public static Servo vibrator;

    //Constants for flicker servo
    private static final double VIBRATOR_CLOSED = 0.4;
    private static final double VIBRATOR_OPEN = 0.58;

    //Constants for height servos
    private static final double ARM_DOWN = 0.7;
    private static final double ARM_UP = 0.35;

    //States
    private static ARM_STATE currentArmState = ARM_STATE.IN;
    private static HEIGHT_STATE currentHeightState = HEIGHT_STATE.DOWN;

    private enum ARM_STATE{
        OUT_FAST,
        OUT_SLOW,
        IN_FAST,
        IN_SLOW,
        OUT,
        IN;
    }

    private enum HEIGHT_STATE{
        UP,
        DOWN;
    }

    //Constructor
    public Arm(){}

    public static void initArm(HardwareMap hwm){
        //Declare Motors on hardware map
        arm = hwm.get(DcMotor.class, "arm");

        heightServo1 = hwm.get(Servo.class, "heightServo1");
        heightServo2 = hwm.get(Servo.class, "heightServo2");
        vibrator = hwm.get(Servo.class, "vibrator");
        //Reverse Motors

        //Init Servos
        heightServo1.setPosition(ARM_DOWN);
        heightServo2.setPosition(ARM_DOWN);
        vibrator.setPosition(VIBRATOR_CLOSED);
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

    public static ARM_STATE getArmState(){
        return currentArmState;
    }

    //Changes the state of the intake based on the input
    public static void heightChangeState(){
        if(currentHeightState == HEIGHT_STATE.DOWN){
            currentHeightState = HEIGHT_STATE.UP;
        }
        else if(currentHeightState == HEIGHT_STATE.UP){
            currentHeightState = HEIGHT_STATE.DOWN;
        }
    }

    public static void armChangeState(String speed){
        if(currentArmState == ARM_STATE.IN && speed.equals("SLOW")){
            currentArmState = ARM_STATE.OUT_SLOW;
        }
        else if(currentArmState == ARM_STATE.IN && speed.equals("FAST")){
            currentArmState = ARM_STATE.OUT_FAST;
        }
        else if(currentArmState == ARM_STATE.OUT && speed.equals("SLOW")){
            currentArmState = ARM_STATE.IN_SLOW;
        }
        else if(currentArmState == ARM_STATE.OUT && speed.equals("FAST")){
            currentArmState = ARM_STATE.IN_FAST;
        }
        else if(currentArmState == ARM_STATE.OUT_SLOW || currentArmState == ARM_STATE.OUT_FAST){
            currentArmState = ARM_STATE.OUT;
        }
        else if(currentArmState == ARM_STATE.IN_SLOW || currentArmState == ARM_STATE.IN_FAST){
            currentArmState = ARM_STATE.IN;
        }
    }

    //Updates the powers for the intake based on the states above
    public static void heightUpdatePosition() {
        if(currentHeightState == HEIGHT_STATE.DOWN){
            armDown();
        }
        else if (currentHeightState == HEIGHT_STATE.UP){
            armUp();
        }
    }

    public static void armUpdatePosition(){
        if(currentArmState == ARM_STATE.IN){
            stopArm();
        }
        else if(currentArmState == ARM_STATE.OUT){
            stopArm();
        }
        else if(currentArmState == ARM_STATE.OUT_SLOW){
            slideArm(.2);
        }
        else if(currentArmState == ARM_STATE.OUT_FAST){
            slideArm(-.6);
        }
        else if(currentArmState == ARM_STATE.IN_SLOW){
            slideArm(-.2);
        }
        else if(currentArmState == ARM_STATE.IN_FAST){
            slideArm(.6);
        }
    }
}
