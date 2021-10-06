package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    //Declare Motors
    public static DcMotor leftFront;
    public static DcMotor leftBack;
    public static DcMotor rightFront;
    public static DcMotor rightback;

    //Constructor
    public DriveTrain(){}

    //Initialize
    public static void initDriveTrain(HardwareMap hwm){
        //Declare Motors on hardware map
        leftFront = hwm.get(DcMotor.class, "leftFront");
        rightFront = hwm.get(DcMotor.class, "rightFront");

        //Reverse Motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}
