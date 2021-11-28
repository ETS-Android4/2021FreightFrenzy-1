package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    public static DcMotor carouselMotor;

    private static double carouselPower = 0.0;

    public Carousel(){};

    public static void initCarousel(HardwareMap hwm){
        //Declare Motors on hardware map
        carouselMotor = hwm.get(DcMotor.class, "carouselMotor");

        //Reverse Motors
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void accelerate(double yValue) {
        double power = Math.pow(yValue, 2);
        if(yValue < 0) {
             power = -power;
        }
        carouselMotor.setPower(power);
    }

    public static void adjustUp(){
        carouselPower += .01;
    }

    public static void adjustDown(){
        carouselPower -= .01;
    }

    public static void powerMotor(){
        carouselMotor.setPower(carouselPower);
    }

    public static void reverseMotor() throws InterruptedException{
        carouselMotor.setPower(-1);
        Thread.sleep(50);
        carouselMotor.setPower(0);
    }

    public static double getCarouselPower(){return carouselPower;}
}
