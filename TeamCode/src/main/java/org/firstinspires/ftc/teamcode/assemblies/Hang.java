package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class Hang {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public AxonHang hangServos;
    //public Servo pulley_left;
    //public Servo pulley_right;
    public Servo hook_grabber;
    public DcMotorEx hang_Right;
    public DcMotorEx hang_Left;



    public AtomicBoolean hangMoving = new AtomicBoolean(false);
    public boolean details = true;

    public static int HANG_EXTEND = 3100;
    public static int HANG_ENGAGE = 800;
    public static int HANG_VELOCITY = 2800;
    public static int HANG_REDUCED_VELOCITY_L = 2400;
    public static int HANG_VELOCITY_R = 2800;

    public static float PULLEYLEFT_STOW = 0.31f;
    public static float PULLEYLEFT_HANG = 0.42f;
    public static float PULLEYLEFT_EXTEND = 0.75f;

    public static float PULLEYRIGHT_STOW = 0.77f;
    public static float PULLEYRIGHT_HANG = 0.67f;
    public static float PULLEYRIGHT_EXTEND = 0.35f;


    public static float HOOKGRABBER_STOW = .498f; // Torque was .025f
    public static float HOOKGRABBER_GRAB = 0.471f; // Torque was 0.08f
    public static float HOOKGRABBER_READY = 0.44f; // Torque was 0.2f
    public static float HOOKGRABBER_DEPLOY = 0.221f; // Torque was 0.86f
    public static float HOOKGRABBER_PRE_RELEASE = 0.35f; // Torque was 0.79f
    public static int HANG_STARTUP_SPINDLE_PAUSE = 250;
    public static int HOOKS_RELEASED= 2500;//was 2818
    public static int HANG_TENSION_THRESHOLD= 200;
    public static int HANG_STOWED_ON_WAY_UP= 3410;
    public static int HOOK_ARM_MOVED_ON_WAY_UP = 6136;
    public static int BUCKET_ROTATE_DURING_TENSION = 2500;
    public static int STOW_FLIPPER = 5000;

    public static int HANG_TENSION_L = 2122;//was 1600
    public static int HANG_TENSION_R = 3400;//was 3222  was 2798
    public static int HANG_LEVEL_3_L = 8024;//5884
    public static int HANG_LEVEL_3_R = 9325;//6838

    public static int HANG_LEVEL_2_L = 3300;//
    public static int HANG_LEVEL_2_R = 4600;//


    public static int HANG_LEVEL_3 = 7000;
    public static int HANG_PAUSE_FOR_LEFT =250;

    public static long HANG_PHASE_2_ENGAGE_PAUSE = 500;
    public static int HANG_PHASE_2_SLACK_PAUSE = 500;
    public static int SLACK_LEVEL = 1000;
    public static int HANG_PHASE_2_PLACE_PAUSE = 1;
    public static int AUTO_LIFT_LEVEL = 5000;

    public static long ENGAGE_HANG_PAUSE_R = 1250;
    public static long ENGAGE_HANG_PAUSE_L = 200;

    public static int HANG_LEFT_INTERMEDIATE = 6512;
    public static int HANG_RIGHT_INTERMEDIATE = 7614;

    public static double HANG_HOLD_POWER = 0.1;
    public boolean hanging = false;
    public boolean hangingL = false;
    public boolean hangingR = false;
    int startString = 0;
    public static float JOYSTICK_Y_DEADBAND = .1f;
    public static float JOYSTICK_X_DEADBAND = .7f;

    public static double STRINGS_TENSIONED = 2000;
    public boolean stringsTensioned = false;



    public Hang() {
        teamUtil.log("Constructing Hang");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Hang");
        hangServos = new AxonHang();
        hangServos.init(hardwareMap);
        //pulley_left = hardwareMap.get(Servo.class,"pulleyleft");
        //pulley_right = hardwareMap.get(Servo.class,"pulleyright");
        hook_grabber = hardwareMap.get(Servo.class,"hookgrabber");
        ((PwmControl)hook_grabber).setPwmRange(new PwmControl.PwmRange(500,2500)); // expand range on GoBilda Torque Servo

        hang_Left = hardwareMap.get(DcMotorEx.class,"hangLeft");
        hang_Right = hardwareMap.get(DcMotorEx.class,"hangRight");
        hang_Left.setTargetPosition(hang_Left.getCurrentPosition());
        hang_Right.setTargetPosition(hang_Right.getCurrentPosition());
        startString = hang_Left.getCurrentPosition();
        hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang_Left.setDirection(DcMotorSimple.Direction.REVERSE);

        stringsTensioned = false;

    }

    public static float HANG_SERVOS_CALIBRATE_POWER = .15f;
    public void calibrate(){
        stowHookGrabber();
        hangServos.calibrate(HANG_SERVOS_CALIBRATE_POWER);
    }



    public void outputTelemetry(){
        telemetry.addLine("Hang Spindles: " +  hang_Left.getCurrentPosition() + ", "+ hang_Right.getCurrentPosition());
        telemetry.addLine("Hang Servos: " +  hangServos.getLPosition() + ", " + hangServos.getRPosition() + "  " + hangServos.lPotentiometer.getVoltage() + ", " + hangServos.rPotentiometer.getVoltage());
    }

    public void stowHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_STOW);
    }

    public void grabHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_GRAB);
    }

    public void readyHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_READY);
    }

    public void deployHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_DEPLOY);
    }
    public void preReleaseHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_PRE_RELEASE);
    }


    public void extendHang(){
        if (details) teamUtil.log("Extending Hang");
        hangMoving.set(true);
        hangServos.runToTarget(hangServos.HANG_L_EXTEND, hangServos.HANG_R_EXTEND, false, 2000);
        //pulley_left.setPosition(PULLEYLEFT_EXTEND);
        //pulley_right.setPosition(PULLEYRIGHT_EXTEND);
        teamUtil.log("Hang Extended");
        hangMoving.set(false);
    }

    public void extendHangNoWait() {
        if (hangMoving.get()) {
            teamUtil.log("WARNING: Attempt to extendHang while Hang is moving--ignored");
            return;
        } else {
            hangMoving.set(true);
            teamUtil.log("Launching Thread to extendHang");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    extendHang();
                }
            });
            thread.start();
        }
    }
    public void engageHang(){
        if (details) teamUtil.log("Engaging Hang");
        hangMoving.set(true);
        hangServos.runToTarget(hangServos.HANG_L_ENGAGE, hangServos.HANG_R_ENGAGE, true, 2000);
        //pulley_left.setPosition(PULLEYLEFT_HANG);
        //pulley_right.setPosition(PULLEYRIGHT_HANG);
        teamUtil.log("Hang Engaged");
        hangMoving.set(false);
    }
    public void engageHangV2(){
        hangServos.lServo.setPower(AxonHang.RTP_MAX_VELOCITY);
        hangServos.rServo.setPower(AxonHang.RTP_MAX_VELOCITY);
        teamUtil.pause(ENGAGE_HANG_PAUSE_R);
        hangServos.rServo.setPower(AxonHang.HOLD_POWER);
        teamUtil.pause(ENGAGE_HANG_PAUSE_L);
        hangServos.lServo.setPower(AxonHang.HOLD_POWER);


    }

    public void engageHangV2NoWait() {
        if (hangMoving.get()) {
            teamUtil.log("WARNING: Attempt to extendHang while Hang is moving--ignored");
            return;
        } else {
            hangMoving.set(true);
            teamUtil.log("Launching Thread to extendHang");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    engageHangV2();
                }
            });
            thread.start();
        }
    }
    public void engageHangNoWait() {
        if (hangMoving.get()) {
            teamUtil.log("WARNING: Attempt to engageHang while Hang is moving--ignored");
            return;
        } else {
            hangMoving.set(true);
            teamUtil.log("Launching Thread to engageHang");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    engageHang();
                }
            });
            thread.start();
        }
    }
    public void stowHang(){
        if (details) teamUtil.log("Stowing Hang");
        hangMoving.set(true);
        hangServos.runToTarget(hangServos.HANG_L_STOW, hangServos.HANG_R_STOW, false, 2000);
        //pulley_left.setPosition(PULLEYLEFT_STOW);
        //pulley_right.setPosition(PULLEYRIGHT_STOW);

        teamUtil.log("Hang Stowed");
        hangMoving.set(false);
    }
    public void stowHangNoWait() {
        if (hangMoving.get()) {
            teamUtil.log("WARNING: Attempt to stowHang while Hang is moving--ignored");
            return;
        } else {
            hangMoving.set(true);
            teamUtil.log("Launching Thread to stowHang");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    stowHang();
                }
            });
            thread.start();
        }
    }

    public static long HANG_SERVOS_CLEAR_PAUSE = 1000;
    public static float HANG_SERVOS_CLEAR_POWER = .15f;
    public void clearHangServos(){
        hangServos.lServo.setPower(HANG_SERVOS_CLEAR_POWER);
        hangServos.rServo.setPower(HANG_SERVOS_CLEAR_POWER);
        teamUtil.pause(HANG_SERVOS_CLEAR_PAUSE);
        hangServos.rServo.setPower(0);
        hangServos.lServo.setPower(0);
    }

    public void clearHangServosNoWait() {
        teamUtil.log("Launching Thread to clearHangServos");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                clearHangServos();
            }
        });
        thread.start();
    }

    public void joystickDrive(float x, float y) {
        if (Math.abs(y)> JOYSTICK_Y_DEADBAND) { // actively moving pulleys
            hanging = false;
            float unadjustedPower = y * -1;
            float velocityAdjust = 0 ;
            if (Math.abs(x) > JOYSTICK_X_DEADBAND) { // uneven velocity to the two motors
                velocityAdjust = Math.abs(x) - JOYSTICK_X_DEADBAND; // range 0 to (1-deadband)
                velocityAdjust = x > 0 ? velocityAdjust : velocityAdjust * -1;
            }
            hang_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hang_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hang_Left.setVelocity(HANG_VELOCITY * (unadjustedPower + velocityAdjust));
            hang_Right.setVelocity(HANG_VELOCITY * (unadjustedPower - velocityAdjust));
        } else {
            if (!hanging) { // we were just moving so switch to hanging mode
                hanging = true;
                hang_Left.setTargetPosition(hang_Left.getCurrentPosition());
                hang_Right.setTargetPosition(hang_Right.getCurrentPosition());
                hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang_Left.setVelocity(HANG_VELOCITY);
                hang_Right.setVelocity(HANG_VELOCITY);
            } // do nothing if we were already hanging
        }
    }

    public void joystickDriveV2(float x, float y) {
        boolean details = false;
        if (hang_Left.getCurrentPosition()> (STRINGS_TENSIONED-startString)) {
            stringsTensioned = true;
        }
        if (Math.abs(y)> JOYSTICK_Y_DEADBAND && x > JOYSTICK_X_DEADBAND) {// actively moving Left side only
            if (details) teamUtil.log("Joystick Drive V2 moving left side");
            if (!hangingR) { // do nothing if we were already hanging
                hangingR = true; // put right motor into hanging mode
                hang_Right.setTargetPosition(hang_Right.getCurrentPosition());
                hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang_Right.setVelocity(HANG_VELOCITY);
            }
            hangingL = false;
            hang_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hang_Left.setVelocity(HANG_VELOCITY * y * -1 );
        } else if (Math.abs(y)> JOYSTICK_Y_DEADBAND && x < -JOYSTICK_X_DEADBAND) { // actively moving Right side only
            if (details) teamUtil.log("Joystick Drive V2 moving right side");

            if (!hangingL) { // do nothing if we were already hanging
                hangingL = true; // put left motor into hanging mode
                hang_Left.setTargetPosition(hang_Left.getCurrentPosition());
                hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang_Left.setVelocity(HANG_VELOCITY);
            }
            hangingR = false;
            hang_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hang_Right.setVelocity(HANG_VELOCITY * y * -1 );
        } else if (Math.abs(y)> JOYSTICK_Y_DEADBAND) { // Actively moving both motors
            if (details) teamUtil.log("Joystick Drive V2 moving both sides");

            hangingL = false;
            hang_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(y<-0.8){
                hang_Left.setVelocity(4000);

            }else{
                hang_Left.setVelocity(HANG_VELOCITY * y * -1 );

            }
            hangingR = false;
            hang_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(y<-0.8){
                hang_Right.setVelocity(4000);

            }else{
                hang_Right.setVelocity(HANG_VELOCITY * y * -1 );

            }
        } else { // Joystick neutral
            if (details) teamUtil.log("Joystick Drive V2 Neutral");

            if (!hangingL) { // do nothing if we were already hanging
                hangingL = true; // put left motor into hanging mode
                hang_Left.setTargetPosition(hang_Left.getCurrentPosition());
                hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang_Left.setVelocity(HANG_VELOCITY);
            }
            if (!hangingR) { // do nothing if we were already hanging
                hangingR = true; // put right motor into hanging mode
                hang_Right.setTargetPosition(hang_Right.getCurrentPosition());
                hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang_Right.setVelocity(HANG_VELOCITY);
            }
        }
    }
}