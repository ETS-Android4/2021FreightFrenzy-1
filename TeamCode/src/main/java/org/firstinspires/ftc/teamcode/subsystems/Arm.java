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
    private static final double ARM_DOWN = .4;
    private static final double ARM_UP = .6;

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

}
