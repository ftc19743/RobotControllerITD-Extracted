package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class Outtake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo outakewrist;
    public Servo outakearm;
    public AnalogInput outakePotentiometer;
    LED frontLED_red;
    LED frontLED_green;


    public AtomicBoolean outtakeMoving = new AtomicBoolean(false);
    public AtomicBoolean interruptOuttakeThread = new AtomicBoolean(false);

    static public float ARM_UP = 0.22f;
    static public float ARM_DOWN = 0.805f;
    static public float ARM_BUCKET_SAFE = .55f;
    static public float WRIST_GRAB = 0.16f;
    static public float WRIST_RELEASE = .813f;
    static public float ARM_REST = .26f;
    static public float ARM_ENGAGE = 0.2f;
    static public float ARM_SOFT_ENGAGE = 0.19f;

    static public float WRIST_REST = 0.48f;
    static public double POTENTIOMETER_OUTPUT_CLEAR = 2;
    static public double POTENTIOMETER_SAFE = 2.38;
    static public double POTENTIOMETER_RELEASE = 2.566;
    static public double POTENTIOMETER_BUCKET_SAFE = 1.5;
    static public double POTENTIOMETER_ATTACH = 3.15;
    static public double POTENTIOMETER_GRAB = .83;
    static public float ARM_START = 0.85f;
    static public float ARM_LEVEL_ONE_ASCENT = 0.13f;
    static public double POTENTIOMETER_WRIST_DEPLOY = 1.4;
    static public float ARM_LEVEL_THREE_ASCENT = 0.13f;





    public Outtake() {
        teamUtil.log("Constructing Outtake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;

        telemetry = teamUtil.theOpMode.telemetry;

    }

    public void initalize() {
        teamUtil.log("Initializing Outtake");
        outakewrist = hardwareMap.get(Servo.class,"outakewrist");
        outakearm = hardwareMap.get(Servo.class,"outakearm");
        outakePotentiometer = hardwareMap.analogInput.get("outakePotentiometer");
        frontLED_green = hardwareMap.get(LED.class, "front_led_green");
        frontLED_red = hardwareMap.get(LED.class, "front_led_red");
        ledOff();

        teamUtil.log("Intake Outtake");
    }

    public void testWiring() {
        outakearm.setPosition(ARM_UP);
        outakewrist.setPosition(WRIST_GRAB);
    }

    public void ledRed(){
        frontLED_red.on();
        frontLED_green.off();
    }

    public void ledGreen(){
        frontLED_red.off();
        frontLED_green.on();
    }

    public void ledOff(){
        frontLED_red.off();
        frontLED_green.off();
    }

    public void firstCalibrate(){
        outakearm.setPosition(ARM_REST);
        outakewrist.setPosition(WRIST_REST);
        /*
        while(Math.abs(outakePotentiometer.getVoltage()-ARM_REST)>0.1){
        }

         */


    }
    public void secondCalibrate(){
        outakearm.setPosition(ARM_START);
        outakewrist.setPosition(WRIST_REST);


    }
    public void deployArmTeleOP(long timeout){
        ledRed();
        outtakeMoving.set(true);
        outakearm.setPosition(ARM_UP);
        long timeoutTime = System.currentTimeMillis()+timeout;
        teamUtil.log("InterruptOuttakeThread State: " + interruptOuttakeThread.get());

        while(outakePotentiometer.getVoltage()<POTENTIOMETER_WRIST_DEPLOY&&teamUtil.keepGoing(timeoutTime)&&!interruptOuttakeThread.get()){
            teamUtil.pause(30);
        }
        outakewrist.setPosition(WRIST_RELEASE);
        outtakeMoving.set(false);
    }

    public void deployArm(){
        outakearm.setPosition(ARM_UP);
        outakewrist.setPosition(WRIST_RELEASE);
    }

    public void deployArmNoWait(long timeout){
        long timeoutTime = System.currentTimeMillis() + timeout;
        if(outtakeMoving.get()){
            teamUtil.log("Outtake is moving shutting down other thread");
            interruptOuttakeThread.set(true);
        }
        teamUtil.log("Launching Thread to deployArm");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(outtakeMoving.get() && teamUtil.keepGoing(timeoutTime)){
                    teamUtil.pause(10);
                }
                interruptOuttakeThread.set(false);
                deployArmTeleOP(timeout);
            }
        });
        thread.start();
    }

    public void outtakeGrab(){
        outakearm.setPosition(ARM_DOWN);
        outakewrist.setPosition(WRIST_GRAB);

        ledGreen();
    }

    public void outtakeGrabTeleop(long timeout){
        outtakeMoving.set(true);
        long timeOutTime = timeout+System.currentTimeMillis();
        outakearm.setPosition(ARM_DOWN);
        outakewrist.setPosition(WRIST_GRAB);
        teamUtil.log("InterruptOuttakeThread State: " + interruptOuttakeThread.get());

        while(outakePotentiometer.getVoltage()>POTENTIOMETER_GRAB+0.1 && teamUtil.keepGoing(timeOutTime)&&!interruptOuttakeThread.get()){
            teamUtil.pause(10);
        }
        ledGreen();
        outtakeMoving.set(false);
    }

    public void outtakeGrabTeleopNoWait(long timeout){
        long timeOutTime = timeout+System.currentTimeMillis();

        if(outtakeMoving.get()){
            teamUtil.log("Outtake is moving shutting down other thread");
            interruptOuttakeThread.set(true);
        }
        teamUtil.log("Launching Thread to deployArm");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(outtakeMoving.get() && teamUtil.keepGoing(timeOutTime)){
                    teamUtil.pause(10);
                }
                interruptOuttakeThread.set(false);
                outtakeGrabTeleop(timeout);
            }
        });
        thread.start();
    }

    public void outtakeRest(){
        outakearm.setPosition(ARM_REST);
        outakewrist.setPosition(WRIST_REST);
        ledOff();
    }

    public void setArmLevelOneAscent(){
        outakearm.setPosition(ARM_LEVEL_ONE_ASCENT);
        outakewrist.setPosition(WRIST_REST);
    }

    public void outakeTelemetry(){
        telemetry.addLine("Outtake arm voltage: " + outakePotentiometer.getVoltage());
    }
}