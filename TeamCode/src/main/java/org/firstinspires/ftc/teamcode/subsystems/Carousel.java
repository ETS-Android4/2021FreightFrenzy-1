package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    public static DcMotor carouselMotor;

    public static double carouselPower = 0.0;

    public Carousel(){};

    public static void initCarousel(HardwareMap hwm){
        //Declare Motors on hardware map
        carouselMotor = hwm.get(DcMotor.class, "carouselMotor");
    }

    public static void accelerate(double yValue){
         carouselMotor.setPower(Math.pow(yValue, 2));
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
}
