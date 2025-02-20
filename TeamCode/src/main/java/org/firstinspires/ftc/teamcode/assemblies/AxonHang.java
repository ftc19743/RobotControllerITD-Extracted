package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AxonHang {
    public CRServo lServo, rServo;
    AnalogInput lPotentiometer, rPotentiometer;


    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public static boolean details = false;

    public int HANG_L_STOW = 0; //set during calibration
    public int HANG_L_EXTEND = 0; //set during calibration
    public int HANG_L_ENGAGE = 0; //set during calibration
    public int HANG_R_STOW = 0; //set during calibration
    public int HANG_R_EXTEND = 0; //set during calibration
    public int HANG_R_ENGAGE = 0; //set during calibration
    public static float RTP_MAX_VELOCITY = .5f;
    public boolean CALIBRATED = false;

    public int lAxonRotations = 0, rAxonRotations = 0; // The number of full rotations positive or negative the servo has traveled from its center range
    private double lLastDegrees360, rLastDegrees360; // the rotational angle of the servo in degrees last time we checked

    public void init(HardwareMap hardwareMap){
        teamUtil.log("Init AxonHang");
        lServo = hardwareMap.crservo.get("pulleyleft");
        rServo = hardwareMap.crservo.get("pulleyright");
        lPotentiometer = hardwareMap.analogInput.get("liftServoLPotentiometer");
        rPotentiometer = hardwareMap.analogInput.get("liftServoRPotentiometer");
        lLastDegrees360 = getDegrees360(lPotentiometer);
        rLastDegrees360 = getDegrees360(rPotentiometer);
        lAxonRotations=0;
        rAxonRotations=0;
    }


    public static double HANG_SERVO_STALL_THRESHOLD = .05;
    public static int HANG_ENGAGE_OFFSET = 200;
    public static int HANG_LEFT_ENGAGE_ADJUST = 30;
    public static int HANG_STOW_OFFSET = 10;
    public static int HANG_EXTEND_OFFSET = 600;
    public static int CALIBRATION_PAUSE = 100;
    public void calibrate(float power) {
        CALIBRATED = false;
        teamUtil.log("Calibrating Stage 1 Hang");

        double lastPositionL = lPotentiometer.getVoltage();
        double lastPositionR = rPotentiometer.getVoltage();

        lServo.setPower(power);
        rServo.setPower(power);
        teamUtil.pause(250);
        double newPositionL = lPotentiometer.getVoltage();
        double newPositionR = rPotentiometer.getVoltage();

        while (Math.abs(newPositionL - lastPositionL) > HANG_SERVO_STALL_THRESHOLD || Math.abs(newPositionR - lastPositionR) > HANG_SERVO_STALL_THRESHOLD) {
            if (details) teamUtil.log("Calibrating Hang Servos: " + lastPositionL + ", " + lastPositionR );
            if (Math.abs(newPositionL - lastPositionL) <= HANG_SERVO_STALL_THRESHOLD) {
                lServo.setPower(0);
            }
            if (Math.abs(newPositionR - lastPositionR) <= HANG_SERVO_STALL_THRESHOLD) {
                rServo.setPower(0);
            }
            lastPositionL = newPositionL;
            lastPositionR = newPositionR;
            teamUtil.pause(CALIBRATION_PAUSE);
            newPositionL = lPotentiometer.getVoltage();
            newPositionR = rPotentiometer.getVoltage();
        }
        lServo.setPower(0);
        rServo.setPower(0);

        if (details) teamUtil.log("Exited calibration loop: " + lastPositionL + "--" + newPositionL + ", " + lastPositionR + "--" + newPositionR);
        teamUtil.pause(250);

        lastPositionL = lPotentiometer.getVoltage();
        lastPositionR = rPotentiometer.getVoltage();
        lAxonRotations = 0;
        rAxonRotations = 0;
        lLastDegrees360 = getDegrees360(lPotentiometer);
        rLastDegrees360 = getDegrees360(rPotentiometer);

        HANG_L_ENGAGE = getLPosition() + HANG_ENGAGE_OFFSET-HANG_LEFT_ENGAGE_ADJUST;
        HANG_R_ENGAGE = getRPosition() + HANG_ENGAGE_OFFSET;
        HANG_L_STOW = getLPosition() + HANG_STOW_OFFSET;
        HANG_R_STOW = getRPosition() + HANG_STOW_OFFSET;
        HANG_L_EXTEND = getLPosition() + HANG_EXTEND_OFFSET;
        HANG_R_EXTEND = getRPosition() + HANG_EXTEND_OFFSET;
        CALIBRATED = true;
        teamUtil.log("Calibrate Hang Stage 1 Done at: "+ lastPositionL + ", " + lastPositionR);
        teamUtil.log("ENGAGE: "+ HANG_L_ENGAGE + ", " + HANG_R_ENGAGE);
        teamUtil.log("STOW: "+ HANG_L_STOW + ", " + HANG_R_STOW);
        teamUtil.log("EXTEND: "+ HANG_L_EXTEND + ", " + HANG_R_EXTEND);
    }


    public void stopServos() {
        lServo.setPower(0);
        rServo.setPower(0);
    }

    // Return the computed absolute position of the servo by using the number
    // of full rotations plus the current angle of the servo
    public int getLPosition(){
        return (int) (360-getDegrees360(lPotentiometer) + lAxonRotations*360); // left side potentiometer is reversed
    }
    public int getRPosition(){
        return (int) getDegrees360(rPotentiometer) + rAxonRotations*360;
    }

    public static int HANG_SERVO_TARGET_THRESHOLD = 10;
    public static float HOLD_POWER = .1f;

    public void runToTarget (int lTarget, int rTarget, boolean hold, long timeOut) {
        moving.set(true);
        loop();
        teamUtil.log("Axon Hang runToTarget: " + lTarget + ", "+ rTarget);
        long timeoutTime = System.currentTimeMillis()+timeOut;

        double ticsFromTargetL = lTarget- getLPosition();
        double ticsFromTargetR = rTarget- getRPosition();

        boolean movingUpL = ticsFromTargetL > 0;
        boolean movingUpR = ticsFromTargetR > 0;
        boolean leftDone = movingUpL ? ticsFromTargetL <= 0 : ticsFromTargetL >= 0;
        boolean rightDone = movingUpR ? ticsFromTargetR <= 0 : ticsFromTargetR >= 0;

        lServo.setPower(movingUpL  ? RTP_MAX_VELOCITY * -1 : RTP_MAX_VELOCITY); // start moving
        rServo.setPower(movingUpR ? RTP_MAX_VELOCITY * -1 : RTP_MAX_VELOCITY);
        while (teamUtil.keepGoing(timeoutTime) && (!leftDone || !rightDone)) {
            loop();
            if (details)
                teamUtil.log("Tics from Target: " + ticsFromTargetL + ", " + ticsFromTargetR + " Positions: "+ getLPosition()+ ", " + getRPosition());
            if (leftDone) {
                lServo.setPower(hold ? HOLD_POWER : 0);
            }
            if (rightDone) {
                rServo.setPower(hold ? HOLD_POWER : 0);
            }
            ticsFromTargetL = lTarget- getLPosition();
            ticsFromTargetR = rTarget- getRPosition();
            leftDone = movingUpL ? ticsFromTargetL <=0 : ticsFromTargetL >= 0;
            rightDone = movingUpR ? ticsFromTargetR <=0 : ticsFromTargetR >= 0;
            teamUtil.pause(30);
        }
        lServo.setPower(hold ? HOLD_POWER : 0);
        rServo.setPower(hold ? HOLD_POWER : 0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Slider runToTarget TIMED OUT: " + getLPosition() + ", " + getRPosition());
        } else {
            teamUtil.log("Slider runToTarget Finished at : " + getLPosition() + ", " + getRPosition());
        }
    }


    // IMPORTANT:  This method must be called frequently whenever the servo is being moved.
    // It keep track of the servo positions and notices when it wraps around between 0 and 360
    // So that the overall position can be calculated
    public void loop(){
        //teamUtil.log("LOOP CALLED");
        double degrees = getDegrees360(lPotentiometer);
        if(Math.abs(lLastDegrees360-degrees)>180){
            if(degrees>180){ // left potentiometer is reversed!
                lAxonRotations++;
                if (details) teamUtil.log("L Hang Axon rotations Went UP.  Voltage: " + lPotentiometer.getVoltage()+ "AXON Position: " + getLPosition());
            }else{
                lAxonRotations--;
                if (details)teamUtil.log("L Hang Axon Rotations Went DOWN: Voltage: " + lPotentiometer.getVoltage()+ "AXON Position: " + getLPosition());
            }
        }
        lLastDegrees360 = degrees;

        degrees = getDegrees360(rPotentiometer);
        if(Math.abs(rLastDegrees360-degrees)>180){
            if(degrees<180){
                rAxonRotations++;
                if (details) teamUtil.log("R Hang Axon rotations Went UP.  Voltage: " + rPotentiometer.getVoltage()+ "AXON Position: " + getRPosition());
            }else{
                rAxonRotations--;
                if (details)teamUtil.log("R Hang Axon Rotations Went DOWN: Voltage: " + rPotentiometer.getVoltage()+ "AXON Position: " + getRPosition());
            }
        }
        rLastDegrees360 = degrees;
    }

    // Convert from potentiometer reading to degrees
    public double getDegrees360(AnalogInput potentiometer){
        return (potentiometer.getVoltage()*360)/(3.274);
    }

}
