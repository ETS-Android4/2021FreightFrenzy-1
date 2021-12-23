package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    //Motors
    public static DcMotor carouselMotor;

    //Powers
    private static double carouselPower = 0.0;

    private static final double CAROUSEL_POWER = 0.21;

    //States
    private static Carousel.CAROUSEL_STATE currentState = Carousel.CAROUSEL_STATE.OFF;

    private enum CAROUSEL_STATE{
        RED,
        BLUE,
        OFF;
    }

    private static String color = "RED";

    //Constructor
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

    public static void brake() throws InterruptedException{
        carouselMotor.setPower(-1);
        Thread.sleep(50);
        carouselMotor.setPower(0);
    }

    public static void stop() {
        carouselMotor.setPower(0);
    }

    public static void spin(double power){
        carouselMotor.setPower(power);
    }

    public static double getCarouselPower(){return carouselPower;}

    public static void changeColor(String newColor){
        color = newColor;
    }

    public static String getColor(){
        return color;
    }

    public static CAROUSEL_STATE getState(){
        return currentState;
    }

    public static void carouselChangeState() throws InterruptedException{
        if(currentState == Carousel.CAROUSEL_STATE.OFF && color.equals("RED")){
            currentState = Carousel.CAROUSEL_STATE.RED;
        }
        else if (currentState == Carousel.CAROUSEL_STATE.RED){
            currentState = Carousel.CAROUSEL_STATE.OFF;
            brake();
        }
        else if(currentState == Carousel.CAROUSEL_STATE.OFF && color.equals("BLUE")){
            currentState = Carousel.CAROUSEL_STATE.BLUE;
        }
        else if(currentState == Carousel.CAROUSEL_STATE.BLUE){
            currentState = Carousel.CAROUSEL_STATE.OFF;
            brake();
        }
    }

    //Updates the powers for the intake based on the states above
    public static void carouselUpdatePosition() throws InterruptedException{
        if(currentState == Carousel.CAROUSEL_STATE.OFF){
            stop();
        }
        else if (currentState == Carousel.CAROUSEL_STATE.RED){
            spin(CAROUSEL_POWER);
        }
        else if(currentState == Carousel.CAROUSEL_STATE.BLUE){
            spin(-CAROUSEL_POWER);
        }
    }

}
