package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public static Servo heightServo;
    public static CRServo bellsServo;

    public Arm(){};

    public static void initArm(HardwareMap hwm){
        //Declare Motors on hardware map
        heightServo = hwm.get(Servo.class, "heightServo");
        bellsServo = hwm.get(CRServo.class, "bellsServo");
        //Reverse Motors
    }

    public static void moveArm(double newHeight){
        heightServo.setPosition(newHeight);
    }

    public static void spinBells(double power){
        bellsServo.setPower(power);
    }
}
