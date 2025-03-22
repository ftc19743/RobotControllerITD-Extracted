package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public BasicDrive drive;
    public Output output;
    public Outtake outtake;
    public Intake intake;
    public Hang hang;
    public Blinkin blinkin;

    public FiveSpecimenAutoV4 fiveSpecimenAutoV4;

    public static boolean details = false;

    public static boolean AUTO_INTAKE_SAMPLE = true;


    static public long DROP_SAMPLE_OUT_BACK_WITH_FLIPPER_RESET_1 = 500;
    static public long DROP_SAMPLE_OUT_BACK_WITH_FLIPPER_RESET_2 = 300;


    public AtomicBoolean autoUnloadNoWaitDone = new AtomicBoolean(false);

    static public boolean waitingForButtonPress = true;
    public static double OUTAKE_ARM_ENGAGE_VAL = 0;

    /*
    static public boolean AA_DEBUG_AUTO = false;

    public boolean keepGoing() {
        if (!AA_DEBUG_AUTO) return true;
        while (true) {
            if (teamUtil.theOpMode.gamepad1.right_bumper) {
                while (teamUtil.theOpMode.gamepad1.right_bumper)
                    ;
                return true;
            }
            if (teamUtil.theOpMode.gamepad1.left_bumper) {
                while (teamUtil.theOpMode.gamepad1.left_bumper)
                    ;
                return false;
            }
        }
    }
*/

    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        outtake = new Outtake();
        intake = new Intake();
        output = new Output();
        hang = new Hang();
        fiveSpecimenAutoV4 = new FiveSpecimenAutoV4();

        teamUtil.robot = this;

    }

    public void initialize() {
        outtake.initalize();
        drive.initalize();
        output.initalize();
        intake.initialize();
        hang.initalize();
        fiveSpecimenAutoV4.init(this);
    }
    public void initCV (boolean enableLiveView) {
        intake.initCV(enableLiveView);
    }

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        intake.intakeTelemetry();
        hang.outputTelemetry();
        output.outputTelemetry();
    }

    public void calibrate() {
        drive.calibrate();
        hang.calibrate();
        outtake.firstCalibrate();
        intake.calibrate();
        output.calibrate();
        teamUtil.pause(250);
        outtake.secondCalibrate();
    }

    public void resetRobot(){
        outtake.outtakeRest();
        teamUtil.pause(1000);
        intake.goToSafe();
        teamUtil.pause(2000);
        output.outputLoad(4000);
        teamUtil.pause(500);
        outtake.secondCalibrate();
        intake.extendersToPositionMaxVelo(Intake.EXTENDER_UNLOAD,4000);
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MISC


    public boolean dropSampleOutBackAndArmGrab(long timeout){
        //TODO Implement Timeout
        outtake.outakewrist.setPosition(Outtake.WRIST_GRAB);

        dropSampleOutBackWithFlipperReset();
        outtake.outakearm.setPosition(Outtake.ARM_DOWN);


        return true;
    }

    public void dropSampleOutBackAndArmGrabNoWait(long timeout){
        if(output.outputMoving.get()){
            teamUtil.log("WARNING: Attempt to dropSampleOutBackAndArmGrabNoWait while output is moving--ignored");
        }
        else{
            teamUtil.log("Launching Thread to dropSampleOutBackNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    dropSampleOutBackAndArmGrab(timeout);
                }

            });
            thread.start();
        }
    }


    //ONLY FOR TELEOP
    public void goToSampleAndGrabAndLiftToBucket(boolean HighBucket){
        if(HighBucket) {
            if(!intake.goToSampleAndGrabV3(false, false,false)){
                return;
            }
            sampleAutoUnloadHighBucketNoWait(true);

        } else {
            if(!intake.goToSampleAndGrabV3(true, true,false)){
                return;
            }
            output.outputLowBucket(3000);

        }
        //intake.flipperGoToSafe(3000);
    }

    public void goToSampleAndGrabAndLiftToBucketNoWait(boolean highBucket){

        teamUtil.log("hangPhase1NoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                goToSampleAndGrabAndLiftToBucket(highBucket);
            }
        });
        thread.start();

    }

    public void dropSampleOutBackWithFlipperReset(){
        teamUtil.log("DropSampleOutBack");
        output.outputMoving.set(true);

        if (output.outputLiftAtBottom.get() && intake.moving.get()) { // still unloading
            teamUtil.log("Attempt to flip bucket at bottom while intake unload still in progress.  Ignored");
            output.outputMoving.set(false);
            return;
        }

        if(output.outputLiftAtBottom.get()){
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM);
        }else{
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        }

        teamUtil.pause(DROP_SAMPLE_OUT_BACK_WITH_FLIPPER_RESET_1);
        intake.flipper.setPosition(Intake.FLIPPER_SAFE);

        if(output.outputLiftAtBottom.get()){
            output.bucket.setPosition(Output.BUCKET_RELOAD);
            teamUtil.pause((Output.DROP_SAMPLE_TIME_2));
        }
        else{
            output.bucket.setPosition(Output.BUCKET_SAFE);
        }
        output.outputMoving.set(false);
        teamUtil.log("DropSampleOutBack Completed");
    }

    public void dropSampleOutBackWithFlipperResetNoWait(){

        teamUtil.log("dropSampleOutBackWithFlipperResetNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                dropSampleOutBackWithFlipperReset();}
        });
        thread.start();

    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPECIMEN AUTO
    public static int EXTENSION_LIFT_HEIGHT = 300;
    public void AutoReadyToSeek(int sliderPos, int extenderPos, boolean delayFlipper) {
        long timeoutTime = System.currentTimeMillis()+1000;
        if (delayFlipper) {
            while (output.lift.getCurrentPosition() < EXTENSION_LIFT_HEIGHT && teamUtil.keepGoing(timeoutTime)) {
                teamUtil.pause(50);
            }
        }
        // get flipper/wrist in position and turn lights on
        intake.goToSeekNoExtenders();

        // move extender and slider to specified positions
        intake.axonSlider.runToEncoderPositionNoWait(sliderPos, true, 1500); // get slider going
        intake.extendersToPositionMaxVelo(extenderPos,1500);

    }

    public void AutoReadyToSeekNoWait(int sliderPos, int extenderPos, boolean delayFlipper) {
        teamUtil.log("Launching Thread to AutoReadyToSeek");

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(intake.moving.get() && !teamUtil.theOpMode.isStopRequested()){ // wait for intake in other thread to stop before launching this one!
                    teamUtil.pause(10);
                }
                AutoReadyToSeek(sliderPos, extenderPos, delayFlipper);
            }
        });
        thread.start();
    }

    public void autoRetractAllNoWait(boolean unload, long timeOut) {
        teamUtil.log("Launching Thread to retractAll");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                intake.retractAll(unload, timeOut);
            }
        });
        thread.start();
    }

    public void autoRetractAndUnload(boolean unload) {
        intake.retractAll(false,2000);
        if(unload){
            intake.unloadV2(true);
        }
    }
    public void autoRetractAndUnloadNoWait(boolean unload) {
        teamUtil.log("Launching Thread to autGrab");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoRetractAndUnload(unload);
            }
        });
        thread.start();
    }


    public int nextSliderPos = (int) AxonSlider.SLIDER_READY; // default value for a grab
    public int nextExtenderPos = Intake.EXTENDER_AUTO_START_SEEK; // default value for a grab

    public static float G00_MAX_POWER = 1f;
    public static float G00_SAFE_POWER = 0.9f;
    public static boolean G0a_EASIER_PICKUP = true;
    public static boolean G0a_EASIER_PICKUP_CYCLE_2 = false;
    public static int G0a_EASIER_PICKUP_STRAFE_ADJUST1 = 250;
    public static int G0a_EASIER_PICKUP_STRAFE_ADJUST2 = 220;
    public static int G0a_EASIER_PICKUP_PAUSE = 100;

    public static int G0a_FAST_STRAFE_ADJUST = 300;
    public static int G0a_FAST_STRAIGHT_ADJUST1 = 200;
    public static int G0a_FAST_REVERSE_ADJUST = 0;
    public static int G0a_SLOW_STRAFE_ADJUST = 50;

    static public int G18_CYCLE_PLACE_SPECIMEN_1_Y = -70; // was 140
    static public int G18a_CYCLE_PLACE_SPECIMEN_Y = -40; //was 140

    static public int G01_PLACE_SPECIMEN1_X = 590;
    static public int G01_PLACE_AND_GRAB_SPECIMEN1_X = 540;
    static public int G01_PLACE_SPECIMEN1_Y = 20;
    static public float G01_PLACE_AND_GRAB_REVERSE_POWER = .5f;
    static public int G01_SPECIMEN1_PAUSE1 = 200;
    static public int G01_SPECIMEN1_PAUSE2 = 0;
    static public int G01_SPECIMEN1_PAUSE3 = 200;
    static public int G01_SPECIMEN1_PAUSE4 = 100;
    static public float G01_SPECIMEN1_END_POWER = .3f;
    static public float G01_SPECIMEN1_GRAB_POWER = .4f;
    static public float G01_SNUG_TO_SUB_POWER = 1f;
    static public float G01_SNUG_TO_SUB_POWER_2 = .1f;


    public int extenderTicPerTooth = 71;
    public int sliderMMperTooth = 30;
    public boolean successfullyGrabbedSample = false;

    public int extenderTeethToEncoder(int teeth){
        return 84 + teeth*extenderTicPerTooth;

    }public int sliderTeethToEncoder(int teeth){
        return (int) (AxonSlider.SLIDER_READY+ (teeth-2)*AxonSlider.SLIDER_TICS_PER_MM*sliderMMperTooth);

    }

    public boolean placeFirstSpecimenV2(boolean grab) {
        teamUtil.log("Place First Specimen V2");
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        outtake.deployArm();

        OpenCVSampleDetectorV2.signalWithBlinkin = false;
        if (grab) {
            // Deploy intake to specified position
            intake.setTargetColor(teamUtil.alliance == RED ? OpenCVSampleDetectorV2.TargetColor.RED : OpenCVSampleDetectorV2.TargetColor.BLUE);
            outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
            AutoReadyToSeekNoWait(nextSliderPos, nextExtenderPos, false); // move intake out for the grab

            // Drive to submersible and snuggle up against it for the grab
            drive.straightHoldingStrafePower(G00_MAX_POWER, G01_PLACE_AND_GRAB_SPECIMEN1_X, G01_PLACE_SPECIMEN1_Y, 0);
            //drive.driveMotorsHeadingsFRPower(180, 0, G01_PLACE_AND_GRAB_REVERSE_POWER); // reverse motors to decelerate quickly
            //teamUtil.pause(G01_SPECIMEN1_PAUSE3); // give it time to decelerate
            //drive.setMotorPowers(0,0,G01_SPECIMEN1_GRAB_POWER,G01_SPECIMEN1_GRAB_POWER);
            reverseUntilForwardMotionStoppedSpecimen(G00_MAX_POWER,1000);
            alignToSubmersibleSpecimen(G01_SNUG_TO_SUB_POWER,G01_SNUG_TO_SUB_POWER_2,1000);
            drive.stopMotors();


            //outtake.outakearm.setPosition(outtake.ARM_SOFT_ENGAGE); // a little extra nudge to make up for the robot low velocity
            drive.waitForRobotToStop(1000); // Robot stationary before we use the CV
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
            teamUtil.pause(G01_SPECIMEN1_PAUSE4); // Give it just a little bit more time after Pinpoint thinks we are stationary

            // retract set to true so we wait here until fully retracted (could be sped up but we need to be careful not to clip the sub with the intake while leaving!
            successfullyGrabbedSample = intake.autoGoToSampleAndGrabV3(false, false,true,3000); // TODO: <- Specify phase 1 timeout?
            intake.retractAllNoWait(false,3000);
            while (intake.extender.getCurrentPosition()>Intake.EXTENDER_AUTO_RETRACT_THRESHOLD && !teamUtil.theOpMode.isStopRequested()){
                teamUtil.pause(5);
            }
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

            intake.lightsOff();
            teamUtil.log("Finished Place First Specimen V2");
            OpenCVSampleDetectorV2.signalWithBlinkin = true;

            return successfullyGrabbedSample;
        } else {
            drive.straightHoldingStrafePower(G00_MAX_POWER, G01_PLACE_SPECIMEN1_X, G01_PLACE_SPECIMEN1_Y, 0);
            drive.driveMotorsHeadingsFRPower(180, 0, G00_MAX_POWER);
            teamUtil.pause(G01_SPECIMEN1_PAUSE1); // give it time to decelerate
            drive.driveMotorsHeadingsFRPower(0, 0, G01_SPECIMEN1_END_POWER);
            teamUtil.pause(G01_SPECIMEN1_PAUSE2); // give it time to click in (can be zero)
            teamUtil.log("Finished Place First Specimen V2");
            OpenCVSampleDetectorV2.signalWithBlinkin = true;

            return true;
        }
    }

    static public int G30_DELIVER_AND_PICKUP_BACKUP_X = 700;
    static public int G31_DELIVER_AND_PICKUP_Y = -840; //was 550
    static public float G32_DELIVER_AND_PICKUP_POWER = .3f;
    static public int G33_DELIVER_AND_PICKUP_PREPARE_FOR_PICKUP_X = 300;
    static public int G34_DELIVER_AND_PICKUP_X = 75;
    public void deliverFirstSample() {
        // moves robot out of the way of the submersible
        drive.straightHoldingStrafePower(G00_MAX_POWER,G30_DELIVER_AND_PICKUP_BACKUP_X,G02_PLACE_SPECIMEN_Y,0);
        outtake.outtakeGrab();
        intake.flipper.setPosition(Intake.FLIPPER_UNLOAD);

        //moves robot into position to drive forward to grab next specimen
        if (G0a_EASIER_PICKUP) {
            drive.strafeHoldingStraightPower(G00_MAX_POWER,G31_DELIVER_AND_PICKUP_Y+G0a_EASIER_PICKUP_STRAFE_ADJUST1,G33_DELIVER_AND_PICKUP_PREPARE_FOR_PICKUP_X,0);
            drive.stopMotors();
            teamUtil.pause(G0a_EASIER_PICKUP_PAUSE);
        } else {
            drive.strafeHoldingStraightPower(G00_MAX_POWER,G31_DELIVER_AND_PICKUP_Y+G0a_FAST_STRAFE_ADJUST,G33_DELIVER_AND_PICKUP_PREPARE_FOR_PICKUP_X,0);
        }
        //intake.unloadToChuteNoWait(); // drop the sample down the chute

        //moves robot to wall for grab
        drive.straightHoldingStrafePower(G32_DELIVER_AND_PICKUP_POWER,G34_DELIVER_AND_PICKUP_X,G31_DELIVER_AND_PICKUP_Y,0);
        intake.release();

        teamUtil.pause(G28_CYCLE_PICKUP_PAUSE);

    }

    public static int G02_PLACE_SPECIMEN_Y = 0; // was 130
    static public int G06_CLEAR_SUB_X = 600;
    static public int G07_CLEAR_SUB_Y = -800; // was -660
    static public int G09_CLEAR_SAMPLE_X = 1100;
    static public int G10_SAMPLE_1_Y = -1000; // was -840
    static public int G10_SAMPLE_Y_ADJUST = 50;
    static public int G11_DROP_SAMPLE1_X = 580;
    public static int G12_REVERSE_BRAKING_PAUSE1 = 200;
    public static int G12_REVERSE_BRAKING_PAUSE2 = 150;
    static public int G13_SAMPLE_2_Y = -1240; // was -1080
    static public int G13a_DROP_SAMPLE2_X = 500;
    static public int G14_SAMPLE_3_Y = -1470; // was -1280 // also adjusted 50 For better wall contact
    static public int G14a_SAMPLE3_Y_ADJUST = 100; // adjusted 50 for better wall contact
    static public int G14a_DROP_SAMPLE3_X = 550;
    static public int G15_PICKUP_1_X = 150;
    static public int G15_PICKUP_1_Y = -1350; // was -1150
    static public float G16_PICKUP_1_POWER = .3f;
    static public int G17_PICKUP_1_PAUSE = 400;
    public static int G0a_GRAB_SAMPLE_STRAIGHT_ADJUST2 = 360;
    public static float G18_REVERSE_POWER = 1f;

    //Collects all blocks using robot to push them
    public void specimenCollectBlocksV3() {

        long startTime = System.currentTimeMillis();


        teamUtil.log("First Specimen Dropped: "+ (System.currentTimeMillis()-startTime));

        // Back up to clear sub
        drive.straightHoldingStrafePower(G00_MAX_POWER, G06_CLEAR_SUB_X, G02_PLACE_SPECIMEN_Y,0);
        outtake.outtakeGrab();
        // strafe over to clear sub on other side
        drive.strafeHoldingStraightPower(G00_MAX_POWER, G07_CLEAR_SUB_Y+G0a_FAST_STRAFE_ADJUST, G06_CLEAR_SUB_X, 0);

        // drive past samples
        drive.straightHoldingStrafePower(G00_MAX_POWER, G09_CLEAR_SAMPLE_X- G0a_FAST_STRAIGHT_ADJUST1, G07_CLEAR_SUB_Y,0);

        // strafe to sample 1
        drive.strafeHoldingStraightPower(G00_MAX_POWER, G10_SAMPLE_1_Y+G10_SAMPLE_Y_ADJUST, G09_CLEAR_SAMPLE_X, 0);

        // push first sample to observation zone
        drive.straightHoldingStrafePower(G18_REVERSE_POWER, G11_DROP_SAMPLE1_X +G0a_FAST_REVERSE_ADJUST, G10_SAMPLE_1_Y,0);

        // head back out to get 2nd sample
        drive.straightHoldingStrafePower(G00_MAX_POWER, G09_CLEAR_SAMPLE_X- G0a_GRAB_SAMPLE_STRAIGHT_ADJUST2, G10_SAMPLE_1_Y,0);
        drive.strafeHoldingStraightPower(G00_MAX_POWER, G13_SAMPLE_2_Y+G10_SAMPLE_Y_ADJUST, G09_CLEAR_SAMPLE_X, 0);

        // push second sample to observation zone
        drive.straightHoldingStrafePower(G18_REVERSE_POWER, G13a_DROP_SAMPLE2_X +G0a_FAST_REVERSE_ADJUST, G13_SAMPLE_2_Y,0);

        // head back out for 3rd sample
        drive.straightHoldingStrafePower(G00_MAX_POWER, G09_CLEAR_SAMPLE_X- G0a_GRAB_SAMPLE_STRAIGHT_ADJUST2, G13_SAMPLE_2_Y,0);
        drive.strafeHoldingStraightPower(G00_MAX_POWER, G14_SAMPLE_3_Y+G14a_SAMPLE3_Y_ADJUST, G09_CLEAR_SAMPLE_X, 0);

        // push 3rd sample to observation zone and grab 2nd specimen
        drive.straightHoldingStrafePower(G18_REVERSE_POWER, G14a_DROP_SAMPLE3_X + G0a_FAST_REVERSE_ADJUST, G14_SAMPLE_3_Y,0);
        drive.driveMotorsHeadingsFRPower(0, 0, G00_MAX_POWER); // reverse motors for fast deceleration
        teamUtil.pause(G12_REVERSE_BRAKING_PAUSE1); // wait a little bit for decel
        // Move over a bit to ensure 3rd Sample doesn't block us on wall
        drive.strafeHoldingStraightPower(G00_MAX_POWER,G15_PICKUP_1_Y-G10_SAMPLE_Y_ADJUST, G14a_DROP_SAMPLE3_X,0);
        drive.straightHoldingStrafePower(G16_PICKUP_1_POWER,G15_PICKUP_1_X,G15_PICKUP_1_Y,0);
        teamUtil.pause(G17_PICKUP_1_PAUSE);
    }


    public void reverseUntilForwardMotionStoppedSpecimen(float power, long timeout) {
        teamUtil.log("reverseUntilForwardMotionStoppedSpecimen");
        long startTime = System.currentTimeMillis();
        drive.odo.update();
        while (drive.odo.getVelX() > 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.driveMotorsHeadingsFRPower(180,0,power);
            drive.odo.update();
            if (details) {
                teamUtil.log("reversing: xPos: " + drive.odo.getPosX() + " xVel: " + drive.odo.getVelX());
            }
        }
        teamUtil.log("reverseUntilForwardMotionStoppedSpecimen Finished at " + (System.currentTimeMillis()-startTime) + "ms. xPos: " + drive.odo.getPosX() + " xVel: " + drive.odo.getVelX());
    }

    public void alignToSubmersibleSpecimen(float power1, float power2, long timeout) {
        teamUtil.log("allignToSubmersibleSpecimen");
        long startTime = System.currentTimeMillis();
        drive.odo.update();
        while (drive.odo.getVelX() < 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.driveMotorsHeadingsFRPower(0,0,power1);
            drive.odo.update();
            if (details) {
                teamUtil.log("reversing: xPos: " + drive.odo.getPosX() + " xVel: " + drive.odo.getVelX());
            }
        }
        teamUtil.pause(10);
        while (drive.odo.getVelX() > 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.driveMotorsHeadingsFRPower(0,0,power2);
            drive.odo.update();
            if (details) {
                teamUtil.log("reversing: xPos: " + drive.odo.getPosX() + " xVel: " + drive.odo.getVelX());
            }
        }
        teamUtil.log("allignToSubmersibleSpecimen Finished at " + (System.currentTimeMillis()-startTime) + "ms. xPos: " + drive.odo.getPosX() + " xVel: " + drive.odo.getVelX());
    }

    public void reverseUntilForwardMotionStoppedSample(double velocity, long timeout) {
        teamUtil.log("reverseUntilForwardMotionStoppedSample");
        long startTime = System.currentTimeMillis();
        drive.driveMotorsHeadingsFR(90,270,velocity);
        drive.odo.update();
        while (drive.odo.getVelY() < 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.odo.update();
            if (details) {
                teamUtil.log("reversing: yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
            }
        }
        teamUtil.log("reverseUntilForwardMotionStoppedSample Finished at " + (System.currentTimeMillis()-startTime) + "ms. yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
    }
    public void forwardUntilForwardMotionStoppedSample(float power, long timeout) {
        teamUtil.log("forwardUntilForwardMotionStoppedSample");
        long startTime = System.currentTimeMillis();
        drive.driveMotorsHeadingsFRPower(270,270,power);
        drive.odo.update();
        while (drive.odo.getVelY() < 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.driveMotorsHeadingsFRPower(270,270,power);
            drive.odo.update();
            if (details) {
                teamUtil.log("reversing: yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
            }
        }
        teamUtil.log("forwardUntilForwardMotionStoppedSample Finished at " + (System.currentTimeMillis()-startTime) + "ms. yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
    }
    public void alignToSubmersibleSample(float power1, float power2, long timeout) {
        teamUtil.log("alignToSubmersibleSample");
        long startTime = System.currentTimeMillis();
        drive.odo.update();
        while (drive.odo.getVelY() > 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.driveMotorsHeadingsFRPower(270,270,power1);
            drive.odo.update();
            if (details) {
                teamUtil.log("Stopping Bounce: yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
            }
        }
        teamUtil.pause(10);
        while (drive.odo.getVelY() < 0 && !teamUtil.theOpMode.isStopRequested()) {
            drive.driveMotorsHeadingsFRPower(270, 270,power2);
            drive.odo.update();
            if (details) {
                teamUtil.log("Aligning: yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
            }
        }
        teamUtil.log("alignToSubmersibleSample Finished at " + (System.currentTimeMillis()-startTime) + "ms. yPos: " + drive.odo.getPosY() + " yVel: " + drive.odo.getVelY());
    }

    static public float G18b_ADJUSTED_MAX_DECLINATION = 35;
    static public int G19_CYCLE_MIDFIELD_X = 500; // was 550 when we were trying for 6
    static public int G20_CYCLE_SPECIMEN_Y_ADJUSTMENT = 25;
    static public int G21_CYCLE_PLACE_SAMPLE_X = 670;
    static public int G21_CYCLE_PLACE_SAMPLE_X_CYCLE_2 = 690;
    static public int G22_CYCLE1_WRIST_CALLBACK = -450;
    static public int G22b_CYCLE2_WRIST_CALLBACK = -800;
    static public int G22c_CYCLE345_WRIST_CALLBACK = -600;

    static public int G23_CYCLE_PLACE_SPECIMEN_PAUSE = 200;
    static public int G23b_CYCLE_REVERSE_PLACE_SPECIMEN_PAUSE = 200;
    static public int G24_CYCLE_BACKUP_X = 700;
    static public int G24_CYCLE_SHIFT_X = 780;
    static public int G24_CYCLE_BACKUP_SHIFT_X = 650;

    static public int G25_CYCLE_PICKUP_Y = -710; //was -690
    static public float G25_CYCLE_PICKUP_POWER = .3f;
    static public int G26_CYCLE_PREPARE_FOR_PICKUP_X = 300;
    static public int G26a_CYCLE_PICKUP_X = 75;
    static public int G28_CYCLE_PICKUP_PAUSE = 200;
    static public long G29_AUTO_MOMENTUM_PAUSE = 200;
    static public long G32_CYCLE_PICKUP_Y_SPECIAL = -790;
    static public long G19_CYCLE_MIDFIELD_2_X = 400;

    public boolean specimenCyclePlace(int cycle, int cycleYTarget) {
        outtake.deployArm();
        float strafeMaxDeclination = BasicDrive.STRAFE_MAX_DECLINATION; // save for later
        BasicDrive.STRAFE_MAX_DECLINATION = G18b_ADJUSTED_MAX_DECLINATION;
        drive.strafeHoldingStraightPower(G00_MAX_POWER, cycleYTarget - G0a_FAST_STRAFE_ADJUST, cycle == 2 ? G19_CYCLE_MIDFIELD_2_X: G19_CYCLE_MIDFIELD_X, 0,
                new BasicDrive.ActionCallback() {
                    @Override
                    public void action() {
                        outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
                    }
                }, cycle == 1? G22_CYCLE1_WRIST_CALLBACK:(cycle==2?G22b_CYCLE2_WRIST_CALLBACK : G22c_CYCLE345_WRIST_CALLBACK), 5000);
        BasicDrive.STRAFE_MAX_DECLINATION = strafeMaxDeclination;

        // Drive straight at sub reversing motors when we get close to decelerate
        drive.straightHoldingStrafePower(G00_MAX_POWER, cycle == 2 ?G21_CYCLE_PLACE_SAMPLE_X_CYCLE_2:G21_CYCLE_PLACE_SAMPLE_X, cycleYTarget, 0);
        reverseUntilForwardMotionStoppedSpecimen(G00_MAX_POWER,1000);

        /*  Replaced by velocity sensing reverse above
        if(cycle>=2){ //  If we stick with grabbing a sample on first place, this should be ">3" as cycle 2 will be the one from deep in the observation zone
            drive.straightHoldingStrafePower(G00_MAX_POWER, G21b_CYCLE_PLACE_SAMPLE_X, cycleYTarget, 0);
            drive.driveMotorsHeadingsFR(180,0,BasicDrive.MAX_VELOCITY);
            teamUtil.pause(G23b_CYCLE_REVERSE_PLACE_SPECIMEN_PAUSE); // give it time to decelerate
        } else{
            drive.straightHoldingStrafePower(G00_MAX_POWER, G21_CYCLE_PLACE_SAMPLE_X, cycleYTarget, 0);
            drive.driveMotorsHeadingsFR(180,0,BasicDrive.MAX_VELOCITY);
            teamUtil.pause(G23b_CYCLE_REVERSE_PLACE_SPECIMEN_PAUSE); // give it time to decelerate and coast to submersible
        }

         */
        return true;
    }

    public boolean specimenCycleV4(int cycle, int cycleYTarget, boolean shiftSpecimens, int shiftYTarget, boolean getNextSpecimen){
        long startTime = System.currentTimeMillis();
        outtake.outakearm.setPosition(Outtake.ARM_UP);

        specimenCyclePlace(cycle, cycleYTarget);

        if (shiftSpecimens) {
            drive.strafeHoldingStraightPower(G00_MAX_POWER, shiftYTarget, G24_CYCLE_SHIFT_X,0);
        }

        if(getNextSpecimen) {
            // moves robot out of the way of the submersible
            if (shiftSpecimens) {
                drive.straightHoldingStrafePower(G00_MAX_POWER,G24_CYCLE_BACKUP_SHIFT_X, shiftYTarget,0);
            } else {
                drive.straightHoldingStrafePower(G00_MAX_POWER,G24_CYCLE_BACKUP_X, G02_PLACE_SPECIMEN_Y,0);
            }
            outtake.outtakeGrab();

            //moves robot into position to drive forward to grab next specimen
            if (G0a_EASIER_PICKUP || (G0a_EASIER_PICKUP_CYCLE_2 && cycle==2)) {
                if (shiftSpecimens) {
                    drive.strafeHoldingStraightPower(G00_MAX_POWER, G25_CYCLE_PICKUP_Y + G0a_EASIER_PICKUP_STRAFE_ADJUST2, G26_CYCLE_PREPARE_FOR_PICKUP_X, 0);
                } else {
                    drive.strafeHoldingStraightPower(G00_MAX_POWER, G25_CYCLE_PICKUP_Y + G0a_EASIER_PICKUP_STRAFE_ADJUST1, G26_CYCLE_PREPARE_FOR_PICKUP_X, 0);
                }
                drive.stopMotors();
                teamUtil.pause(G0a_EASIER_PICKUP_PAUSE);
            } else {
                drive.strafeHoldingStraightPower(G00_MAX_POWER, G25_CYCLE_PICKUP_Y + G0a_FAST_STRAFE_ADJUST, G26_CYCLE_PREPARE_FOR_PICKUP_X, 0);
            }
            //moves robot to wall for grab
            drive.straightHoldingStrafePower(G25_CYCLE_PICKUP_POWER,G26a_CYCLE_PICKUP_X,G25_CYCLE_PICKUP_Y,0);
            teamUtil.pause(G28_CYCLE_PICKUP_PAUSE);
        }
        BasicDrive.MIN_STRAFE_START_VELOCITY = 500;
        BasicDrive.MIN_START_VELOCITY = 300;

        long elapsedTime = System.currentTimeMillis()-startTime;
        teamUtil.log("Cycle Time: "+elapsedTime);
        return true;
    }


    // Work in progress on 6 specimen auto
    static public int[] G33_6_CYCLE_Y_PLACEMENTS = {20, 130, 20, 75, 130}; // was {0, 68, 112, 145, 178}
    static public int G33_6_CYCLE_SHIFT_2 = 70;
    static public int G33_6_TIME_FOR_LAST_SPECIMEN = 28500;
    static public int G33_7_TIME_FOR_PARK = 28500;

    public boolean autoV5Specimen() {
        teamUtil.log("Running Specimen Auto V5.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));
        teamUtil.startTime = System.currentTimeMillis();

        drive.setRobotPosition(0, 0, 0);

        placeFirstSpecimenV2(true);
        deliverFirstSample();
        specimenCyclePlace(1, G33_6_CYCLE_Y_PLACEMENTS[0]); //second specimen delivered
        specimenCollectBlocksV3();

        specimenCycleV4(2, G33_6_CYCLE_Y_PLACEMENTS[1], true, G33_6_CYCLE_SHIFT_2, true); //third specimen delivered
        specimenCycleV4(3, G33_6_CYCLE_Y_PLACEMENTS[2], false, 0, true); //fourth specimen delivered
        specimenCycleV4(4, G33_6_CYCLE_Y_PLACEMENTS[3], false, 0, true); //fifth specimen delivered
        if (successfullyGrabbedSample) {
            if (System.currentTimeMillis() - teamUtil.startTime < G33_6_TIME_FOR_LAST_SPECIMEN) {
                specimenCycleV4(5, G33_6_CYCLE_Y_PLACEMENTS[4], false, 0, false); //sixth specimen delivered
            } else {
                teamUtil.log("No Time for Last Cycle");
            }
            if (System.currentTimeMillis() - teamUtil.startTime < G33_7_TIME_FOR_PARK) {
                park();
            } else {
                teamUtil.log("No Time for Park");
                drive.driveMotorsHeadingsPower(180,0,1);
                teamUtil.pause(Z_FINAL_DROP_MOVE_TIME);
                drive.stopMotors();
            }
        }
        drive.stopMotors();

        //TODO make it so AUTO_INTAKE_SPECIMEN is an option when initializing
        return true;
    }

    static public int G34_PARK_X = 130;
    static public int G34_PARK_Y = -577;
    static public int G34_PARK_DH = 255;

    public void park(){
        drive.straightHoldingStrafePower(G00_MAX_POWER,G24_CYCLE_BACKUP_X,G02_PLACE_SPECIMEN_Y,0);
        outtake.outtakeGrab();

        drive.moveToY(G00_MAX_POWER,G34_PARK_Y,G34_PARK_DH,(int) drive.adjustAngle(G34_PARK_DH+180));
        drive.stopMotors();
    }

    // 5 specimen auto as ran at League Champs
    public boolean autoV4Specimen(){
        return fiveSpecimenAutoV4.autoV4Specimen();
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HANG
    public static int PICK_UP_HOOKS_PAUSE_1 = 1000;
    public static int PICK_UP_HOOKS_PAUSE_2 = 300;
    public static int PICK_UP_HOOKS_PAUSE_3 = 250;
    public static int PICK_UP_HOOKS_PAUSE_4 = 500;
    public static int READY_TO_PLACE_HOOKS_PAUSE_1 = 250;
    public static int READY_TO_PLACE_HOOKS_VELOCITY = 1400;
    public static int PLACE_HOOKS_VELOCITY = 400;



    public void hangPhase1(){
        hang.extendHangNoWait();
        output.bucket.setPosition(Output.BUCKET_HANG);
        pickUpHooks();
        readyToPlaceHooks();
        intake.extender.setTargetPosition(Intake.EXTENDER_CALIBRATE);
        intake.extender.setVelocity(Intake.EXTENDER_HOLD_RETRACT_VELOCITY);
    }

    public void hangPhase1NoWait(){

        teamUtil.log("hangPhase1NoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                hangPhase1();
            }
        });
        thread.start();

    }


    public void hangPhase2V3(){
        long timeOutTime = System.currentTimeMillis() + 8000;
        hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Assumes 33.5" of string out
        hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Assumes 39.5" of string out

        hang.engageHangV2NoWait();
        teamUtil.pause(Hang.HANG_PHASE_2_ENGAGE_PAUSE); // don't put hooks on bar until we are off of ground
        output.lift.setVelocity(Robot.PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_AT_BAR);

        teamUtil.log("Tensioning Strings");
        hang.hang_Left.setTargetPosition(Hang.HANG_TENSION_L); // Start tensioning strings while hooks are being placed--there is enough slack
        hang.hang_Right.setTargetPosition(Hang.HANG_TENSION_R);
        hang.hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Left.setVelocity(Hang.HANG_VELOCITY);
        hang.hang_Right.setVelocity(Hang.HANG_VELOCITY);

        // Wait until we are in the right position to leave the barrier at the bottom
        while (teamUtil.keepGoing(timeOutTime) &&
                (Hang.HANG_TENSION_L - hang.hang_Left.getCurrentPosition() > 50 ||
                 Hang.HANG_TENSION_R - hang.hang_Right.getCurrentPosition() > 50 ))  {
            hangPhase2DelayedOps();
            if (details) {teamUtil.log("Hangleft: " + hang.hang_Left.getCurrentPosition()+ " HangRight: "+ hang.hang_Right.getCurrentPosition());}
            teamUtil.pause(50);
        }
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Hang Phase 2 V3 Timed Out!");
            return;
        }

        // Head to the top
        teamUtil.log("Heading Up");
        hang.hang_Right.setTargetPosition(Hang.HANG_LEVEL_3_R);
        hang.hang_Left.setTargetPosition(Hang.HANG_LEFT_INTERMEDIATE);
        //hang.hang_Left.setTargetPosition(Hang.HANG_LEVEL_3_L);

        teamUtil.log("Both Joystick Drive Booleans HANGINGL and HANGINGR set true in hang Phase 2");
        hang.hangingL = true; hang.hangingR = true;// fake out control code to let it go up automatically until someone touches the joystick
    }

    public void hangPhase2Level2(){
        long timeOutTime = System.currentTimeMillis() + 8000;
        hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Assumes 33.5" of string out
        hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Assumes 39.5" of string out

        hang.stowHangNoWait();
        output.lift.setVelocity(Robot.PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_AT_BAR);

        teamUtil.log("Tensioning Strings");
        hang.hang_Left.setTargetPosition(Hang.HANG_LEVEL_2_L); // Start tensioning strings while hooks are being placed--there is enough slack
        hang.hang_Right.setTargetPosition(Hang.HANG_LEVEL_2_R);
        hang.hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Left.setVelocity(Hang.HANG_VELOCITY);
        hang.hang_Right.setVelocity(Hang.HANG_VELOCITY);


        teamUtil.log("Both Joystick Drive Booleans HANGINGL and HANGINGR set true in hang Phase 2");
        hang.hangingL = true; hang.hangingR = true;// fake out control code to let it go up automatically until someone touches the joystick


    }

    boolean evenedStringsOut = false;
    public void evenOutStringsWhenNeeded(){
        if(hang.hang_Right.getCurrentPosition()>Hang.HANG_RIGHT_INTERMEDIATE&&!evenedStringsOut){
            evenedStringsOut=true;
            teamUtil.log("Strings evened out");
            hang.hang_Left.setTargetPosition(Hang.HANG_LEVEL_3_L);
        }
    }


    public void dropLift() {
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_PRE_RELEASE);
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY); // Run to near bottom
        output.lift.setTargetPosition(Output.LIFT_DOWN);
        while (output.lift.getCurrentPosition() > Output.LIFT_DOWN+10 && !teamUtil.theOpMode.isStopRequested()) {
            teamUtil.pause(50);
        }
        output.lift.setVelocity(0); // Turn off lift motor at bottom
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM); // rotate bucket to avoid string while tensioning
    }

    boolean liftDropped = false;
    public void dropLiftWhenNeeded() {
        if (hang.hang_Left.getCurrentPosition() > Hang.HOOKS_RELEASED && !liftDropped) {
            liftDropped = true;
            teamUtil.log("Launching Thread to drop Lift");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    dropLift();
                }
            });
            thread.start();
        }
    }



    boolean hangStowed = false;
    public void stowHangWhenNeeded() {
        if (hang.hang_Left.getCurrentPosition() > Hang.HANG_STOWED_ON_WAY_UP && !hangStowed) {
            hangStowed = true;
            hang.clearHangServosNoWait();
            hookArmMoved = true;
            hang.deployHookGrabber();
        }
    }

    boolean hookArmMoved = false;
    public void moveHookArmWhenNeeded() {
        if (hang.hang_Left.getCurrentPosition() > Hang.HOOK_ARM_MOVED_ON_WAY_UP && !hookArmMoved) {
            hookArmMoved = true;
            hang.deployHookGrabber();
        }
    }

    boolean bucketRotated = false;
    public void rotateBucketWhenNeeded() {
        if (!bucketRotated && hang.hang_Right.getCurrentPosition() > Hang.BUCKET_ROTATE_DURING_TENSION ) {
            bucketRotated = true;
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM);
        }
    }

    boolean flipperStowed = false;
    public void stowFlipperWhenNeeded() {
        if (!flipperStowed && hang.hang_Left.getCurrentPosition() > Hang.STOW_FLIPPER ) {
            flipperStowed = true;
            intake.flipperGoToStow();
        }
    }

    public void hangPhase2DelayedOps() {
        rotateBucketWhenNeeded();
        dropLiftWhenNeeded();
        stowFlipperWhenNeeded();
        stowHangWhenNeeded();
        moveHookArmWhenNeeded();
        evenOutStringsWhenNeeded();
    }

    public static float LIFT_PICKUP_HOOKS_POWER_1 = 0.5f;
    public static float LIFT_PICKUP_HOOKS_POWER_2 = -0.5f;

    public void pickUpHooks(){
        intake.flipper.setPosition(Intake.FLIPPER_SAFE);
        intake.grab();
        outtake.outakearm.setPosition(Outtake.ARM_LEVEL_THREE_ASCENT);

        output.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        output.lift.setPower(LIFT_PICKUP_HOOKS_POWER_1);
        while(output.lift.getCurrentPosition()<Output.LIFT_SAFE_FOR_HOOK_HOLDER && !teamUtil.theOpMode.isStopRequested()){
            teamUtil.pause(10);
        }
        output.lift.setPower(0);

        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_2);

        output.lift.setPower(LIFT_PICKUP_HOOKS_POWER_2);
        while(output.lift.getCurrentPosition()>Output.LIFT_PICKUP_FOR_HOOK_HOLDER && !teamUtil.theOpMode.isStopRequested()){
            teamUtil.pause(10);
        }
        output.lift.setPower(0);

        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_GRAB);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_4);

    }

    public void readyToPlaceHooks(){
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(READY_TO_PLACE_HOOKS_PAUSE_1);
        output.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        output.lift.setTargetPosition(Output.LIFT_ABOVE_BAR);
        output.lift.setVelocity(READY_TO_PLACE_HOOKS_VELOCITY);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_DEPLOY);
    }


    public void getReadyToHang() {
        pickUpHooks();
        readyToPlaceHooks();
    }

    public void getReadyToHangNoWait() {
        if (hang.hangMoving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to getReadyToHang while hang is moving--ignored");
            return;
        } else {
            teamUtil.log("Launching Thread to getReadyToHang");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    getReadyToHang();
                }
            });
            thread.start();
        }

    }



//////////////////////////////////////////////////////////////////////////////////////
/////////////////  SAMPLE AUTO

    public static int A00_MAX_SPEED_NEAR_BUCKET = 2500;
    public static int A00_TRANSITION_SPEED = 750;
    public static int A00_END_SPEED = 400;
    public static int A00_END_VELOCITY_FOR_PICKUP = 1000;
    public static int A01_SAMPLE_1_SLIDER = (int)AxonSlider.SLIDER_READY;
    public static int A01_SAMPLE_1_EXTENDER = 1000;
    public static int A02_SAMPLE_2_SLIDER = (int)AxonSlider.SLIDER_READY;
    public static int A02_SAMPLE_2_EXTENDER = 700;
    public static int A03_SAMPLE_3_SLIDER = -17000;
    public static int A03_SAMPLE_3_EXTENDER = 700;
    public static int A03_DROP_TIME = 400;
    public static int A04_READY_FOR_BUCKET_STRAIGHT = 100;
    public static int A04_READY_FOR_BUCKET_STRAFE = 120;
    public static int A04_END_VELOCITY = 1500;
    public static int A05_1_BUCKET_STRAIGHT = 80;
    public static int A05_1_BUCKET_STRAFE = 450;
    public static int A05_1_BUCKET_HEADING = 325;
    public static int A05_1_BUCKET_END_VELOCITY = 800;

    public static int A06_1_SAMPLE_PICKUP_STRAFE = 470; //390
    public static int A06_1_SAMPLE_PICKUP_STRAIGHT = 240;//260
    public static int A06_1_SAMPLE_PICKUP_DH = 340;//347
    public static int A06_1_SAMPLE_PICKUP_RH =340;//347
    public static float A06_1_SAMPLE_PICKUP_POWER = 0.5f;//347
    public static int A06_1_BRAKE_PAUSE = 100;

    public static int A07_2_BUCKET_STRAIGHT = 90;
    public static int A07_2_BUCKET_STRAFE = 500;
    public static int A07_2_BUCKET_HEADING = 325;

    public static int A08_2_SAMPLE_PICKUP_STRAFE = 500;
    public static int A08_2_SAMPLE_PICKUP_STRAIGHT = 260;
    public static int A08_2_SAMPLE_PICKUP_HEADING = 355;
    public static float A08_2_SAMPLE_PICKUP_POWER = 0.25f;

    public static int A09_3_BUCKET_STRAIGHT = 90;
    public static int A09_3_BUCKET_STRAFE = 500;
    public static int A09_3_BUCKET_HEADING = 325;

    public static int A10_3_SAMPLE_PICKUP_STRAFE = 600;
    public static int A10_3_SAMPLE_PICKUP_STRAIGHT = 280;
    public static int A10_3_SAMPLE_PICKUP_HEADING = 0;

    public static int A11_3_PRE_BUCKET_STRAIGHT = 240;
    public static int A11_3_PRE_BUCKET_STRAFE = 490;
    public static int A11_3_PRE_BUCKET_HEADING = 0;
    public static int A12_3_BUCKET_STRAIGHT = 90;
    public static int A12_3_BUCKET_STRAFE = 500;
    public static int A12_3_BUCKET_HEADING = 320;
    public static long A12_SAMPLE_PICKUP_TIMEOUT = 1000;


    public void deliver4Samples() {
        outtakeUpandGoToHighBucketNoWait();
        AutoReadyToSeekNoWait(A01_SAMPLE_1_SLIDER, A01_SAMPLE_1_EXTENDER, true); // move intake out for the grab

        // Move to first drop
        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET, A04_READY_FOR_BUCKET_STRAFE,A04_READY_FOR_BUCKET_STRAIGHT,0,A04_END_VELOCITY,null,0, false,5000);
        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET, A05_1_BUCKET_STRAFE, A05_1_BUCKET_STRAIGHT,A05_1_BUCKET_HEADING,A05_1_BUCKET_END_VELOCITY,null,0, false, 5000);
        drive.setMotorsActiveBrake();

        WaitForLiftReadyToUnloadSampleHighBucket(2000);
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        teamUtil.pause(A03_DROP_TIME);
        autoGoToLoadNoWait(3000);



        // Move to pickup 1st sample
        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A06_1_SAMPLE_PICKUP_STRAFE,A06_1_SAMPLE_PICKUP_STRAIGHT,A06_1_SAMPLE_PICKUP_RH,A00_END_VELOCITY_FOR_PICKUP,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        teamUtil.pause(A06_1_BRAKE_PAUSE);

        // Grab and unload (counting on bucket to be at the bottom by the time we get there!
        boolean grabbedSample=intake.autoGoToSampleAndGrabV3(false,false,true,A12_SAMPLE_PICKUP_TIMEOUT);
        sampleAutoUnloadHighBucketNoWait(false);

        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A07_2_BUCKET_STRAFE,A07_2_BUCKET_STRAIGHT,A07_2_BUCKET_HEADING,A00_END_SPEED,null,0, false, 5000);
        drive.setMotorsActiveBrake();

        AutoReadyToSeekNoWait(A02_SAMPLE_2_SLIDER, A02_SAMPLE_2_EXTENDER, true); // move intake out for the next grab

        WaitForLiftReadyToUnloadSampleHighBucket(2000);
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        teamUtil.pause(A03_DROP_TIME);
        autoGoToLoadNoWait(3000);



        // Move to pickup 2nd sample
        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A08_2_SAMPLE_PICKUP_STRAFE,A08_2_SAMPLE_PICKUP_STRAIGHT,A08_2_SAMPLE_PICKUP_HEADING,A00_END_VELOCITY_FOR_PICKUP,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        teamUtil.pause(A06_1_BRAKE_PAUSE);

        // Grab and unload (counting on bucket to be at the bottom by the time we get there!
        grabbedSample=intake.autoGoToSampleAndGrabV3(false, false,true,A12_SAMPLE_PICKUP_TIMEOUT);
        sampleAutoUnloadHighBucketNoWait(false);

        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A09_3_BUCKET_STRAFE,A09_3_BUCKET_STRAIGHT,A09_3_BUCKET_HEADING,A00_END_SPEED,null,0, false, 5000);
        drive.setMotorsActiveBrake();

        AutoReadyToSeekNoWait(A03_SAMPLE_3_SLIDER, A03_SAMPLE_3_EXTENDER, true); // move intake out for the next grab

        WaitForLiftReadyToUnloadSampleHighBucket(2000);
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        teamUtil.pause(A03_DROP_TIME);

        //drive.straightHoldingStrafePower(A10_3_PRE_SAMPLE_PICKUP_POWER,A10_3_PRE_SAMPLE_PICKUP_STRAIGHT,A10_3_PRE_SAMPLE_PICKUP_STRAFE,A10_3_PRE_SAMPLE_PICKUP_HEADING);
        autoGoToLoadNoWait(3000);



        // Move to pickup 3rd sample
        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A10_3_SAMPLE_PICKUP_STRAFE,A10_3_SAMPLE_PICKUP_STRAIGHT,A10_3_SAMPLE_PICKUP_HEADING,A00_END_VELOCITY_FOR_PICKUP,null,0,false,3000);
        //drive.straightHoldingStrafePower(A10_3_SAMPLE_PICKUP_POWER,A10_3_SAMPLE_PICKUP_STRAIGHT,A10_3_SAMPLE_PICKUP_STRAFE,A10_3_SAMPLE_PICKUP_HEADING);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        teamUtil.pause(A06_1_BRAKE_PAUSE);
        // Grab and unload (counting on bucket to be at the bottom by the time we get there!

        grabbedSample=intake.autoGoToSampleAndGrabV3(false, false,true,A12_SAMPLE_PICKUP_TIMEOUT);
        sampleAutoUnloadHighBucketNoWait(false);

        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A11_3_PRE_BUCKET_STRAFE,A11_3_PRE_BUCKET_STRAIGHT,A11_3_PRE_BUCKET_HEADING,A00_TRANSITION_SPEED,null,0, false, 5000);
        drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A12_3_BUCKET_STRAFE,A12_3_BUCKET_STRAIGHT,A12_3_BUCKET_HEADING,A00_END_SPEED,null,0, false, 5000);
        drive.setMotorsActiveBrake();

        WaitForLiftReadyToUnloadSampleHighBucket(2000);
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        teamUtil.pause(A03_DROP_TIME);
        autoGoToLoadNoWait(3000);
    }




    public static float C00_MAX_POWER = 1f;
    public static float C00_END_POWER = .25f;
    public static float C00_ROTATION_ADJUST_FACTOR_POWER = .022f;

    public static float C00_SUB_POWER = .8f;
    public static float C00_SUB_ALIGN_POWER_1 = .8f;
    public static float C00_SUB_ALIGN_POWER_2 = .2f;
    public static int C01_SUB_HEADING = 270;
    public static int C01_SUB_X1 = 800;
    public static int C01_SUB_X = 880;
    public static int C01_SUB_Y = -230;
    public static int C01_SUB_STALL_PAUSE = 100;
    public static int C01_EXTENDER_POS = 250;
    public static int C01_SLIDER_POS = (int) AxonSlider.SLIDER_READY;

    public static int C02_SUB_BACKUP_Y = -350;
    public static int C02_SUB_BACKUP_HEADING = 90;
    public static int C02_SUB_BASKET_X = 250;
    public static int C02_SUB_BASKET_Y = 420;
    public static int C02_SUB_BASKET_HEADING = 155;
    public static int C02_SUB_BASKET_RH = 315;
    public static int C02_BASKET_BRAKE_PAUSE = 0;
    public static int C02_BASKET_TO_SUB_HEADING = 340;

    public static int S00_SLIDER_CYCLE_ADJUST = -4000;

    public static float C00_MAX_VELOCITY = 2800;
    public static float C00_TRANSITION_VELOCITY = 2200f;
    public static float C00_ROTATION_ADJUST_FACTOR_VELOCITY = 40;
    public static int C03_SUB_X1 = 750;
    public static int C03_SUB_Y1 = 160;
    public static int C03_SUB_HEADING1 = 345;

    public static int C03_SUB_X2 = 1060;
    public static int C03_SUB_Y2 = 0;
    public static int C03_SUB_HEADING2 = 300;

    public static int C03_SUB_X3 = 1320;
    public static int C03_SUB_Y3 = -240;
    public static int C03_SUB_HEADING3 = 270;
    public static float C03_SUB_END_VELOCITY = 2200f;

    public static float C00_SUB_ROTATION_ADJUST_FACTOR_POWER = .02f;
    public static int C03_SUB_STALL_PAUSE = 100;

    public void bucketToSubV3 (int launchIntake) {
        long startTime = System.currentTimeMillis();
        teamUtil.log("---------- BUCKET TO SUB ");

        // Move to sub
        drive.moveTo(C00_MAX_VELOCITY,C03_SUB_Y1,C03_SUB_X1,C03_SUB_HEADING1,C00_TRANSITION_VELOCITY,null,0, false, 1500);
        drive.moveTo(C00_TRANSITION_VELOCITY,C03_SUB_Y2,C03_SUB_X2,C03_SUB_HEADING2,C00_TRANSITION_VELOCITY,null,0, false, 1000);
        if(launchIntake>0) {
            AutoReadyToSeekNoWait(C01_SLIDER_POS+((launchIntake-2)*S00_SLIDER_CYCLE_ADJUST), C01_EXTENDER_POS, false);
        }

        // chill out rotational adjust for these movements
        float savedRotationAdjust = BasicDrive.ROTATION_ADJUST_FACTOR_POWER;
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = C00_SUB_ROTATION_ADJUST_FACTOR_POWER;
        drive.moveToYHoldingLine(C00_SUB_POWER, C03_SUB_Y3, C03_SUB_X3, C03_SUB_HEADING3,C03_SUB_HEADING3, C00_SUB_POWER, null, 0, 1000);
        reverseUntilForwardMotionStoppedSample(C00_SUB_POWER,1000); // hit the barrier while slowing down
        alignToSubmersibleSample(C00_SUB_ALIGN_POWER_1,C00_SUB_ALIGN_POWER_2,1500); // stop the bounce and the align to barrier
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        teamUtil.pause(C01_SUB_STALL_PAUSE); // give it a little more time to get motionless

        // Restore rotational coefficient
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = savedRotationAdjust;
        teamUtil.log("---------- BUCKET TO SUB Finished in : " + (System.currentTimeMillis()-startTime));
    }



    public static int C14_CYCLE_BACK_TO_BUCKET_X = 90;
    public static int C14_CYCLE_BACK_TO_BUCKET_Y = 500;
    public static int C14_CYCLE_BACK_TO_BUCKET_END_VELOCITY = 400;
    public static int C14_CYCLE_BACK_TO_BUCKET_HEADING = 330;
    public void subToBasketV2(boolean useArms) {
        long startTime = System.currentTimeMillis();
        teamUtil.log("---------- SUB TO BUCKET");

        // chill out rotational adjust for these movements
        float savedRotationAdjust = BasicDrive.ROTATION_ADJUST_FACTOR_POWER;
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = C00_ROTATION_ADJUST_FACTOR_POWER;

        if (useArms) {
            // need to modify sampleAutoUnloadHighBucketNoWait to provide a parameter to use "seek" instead of "pre-grab" for this situation (then it should be good for teleop too!)
            sampleAutoUnloadHighBucketNoWait(true); // pull in sample, transfer, and send high bucket to top
        }
        drive.moveToY(C00_MAX_POWER, C02_SUB_BACKUP_Y, C02_SUB_BACKUP_HEADING, C01_SUB_HEADING);
        drive.moveToXHoldingStrafe(C00_MAX_POWER, C02_SUB_BASKET_X, C02_SUB_BASKET_Y, C02_SUB_BASKET_HEADING, C02_SUB_BASKET_RH, C00_END_POWER, null, 0, 2000);
        drive.stopMotors();
        teamUtil.pause(C02_BASKET_BRAKE_PAUSE);

        //drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,C14_CYCLE_BACK_TO_BUCKET_Y,C14_CYCLE_BACK_TO_BUCKET_X,C14_CYCLE_BACK_TO_BUCKET_HEADING,C14_CYCLE_BACK_TO_BUCKET_END_VELOCITY,null,0, false, 5000);
        if (useArms) {
            WaitForLiftReadyToUnloadSampleHighBucket(2000); // wait for lift to be high enough
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP); // unload the sample
            teamUtil.pause(A03_DROP_TIME);
            autoGoToLoadNoWait(3000); // lift back to bottom
        }

        // Restore rotational coefficient
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = savedRotationAdjust;
        teamUtil.log("---------- SUB TO BUCKET Finished in : " + (System.currentTimeMillis()-startTime));
    }

    public void autoAscentPark(){
        long startTime = System.currentTimeMillis();
        teamUtil.log("---------- PARKING");
        bucketToSubV3(0);
        drive.stopMotors();
        outtake.setArmLevelOneAscent();
        teamUtil.log("---------- PARKING Finished in : " + (System.currentTimeMillis()-startTime));
    }

    public static long Z0_CYCLE_GRAB_TIME = 6000;
    public static long Z0_CYCLE_DELIVER_TIME = 3000;
    public static long Z0_PARK_TIME = 3000;
    public static long Z_FINAL_DROP_MOVE_TIME = 250;
    public static long Z_GRAB_TIMEOUT_TIME_BUCKET_CYCLE = 1500;


    public boolean parked = false;
    public boolean bucketCycleV2(long autoStartTime, int position){
        long startTime = System.currentTimeMillis();
        teamUtil.log("---------- CYCLE STARTED ---------------");

        bucketToSubV3(position);
        long grabStartTime = System.currentTimeMillis();
        teamUtil.log("---------- GRABBING");

        if(intake.autoGoToSampleAndGrabV3(false,false,true,Z_GRAB_TIMEOUT_TIME_BUCKET_CYCLE)) {
            teamUtil.log("---------- GRAB Finished in : " + (System.currentTimeMillis() - startTime));

            if (System.currentTimeMillis()-autoStartTime>30000-Z0_CYCLE_DELIVER_TIME) { // not enough time to deliver, so just park and transfer
                outtake.setArmLevelOneAscent();
                intake.autoRetractAllAndUnload(true, System.currentTimeMillis()-autoStartTime);
                teamUtil.log("---------- CYCLE Parked in : " + (System.currentTimeMillis() - startTime));
                parked = true;
                return false; // don't keep cycling
            } else { // deliver to the high bucket
                //sampleAutoUnloadHighBucketNoWait(true); // this is called at the start of subToBasketV2 on the next line!
                subToBasketV2(true);
                WaitForLiftReadyToUnloadSampleHighBucket(2000);
                output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
                teamUtil.pause(A03_DROP_TIME);
                autoGoToLoadNoWait(3000);
                teamUtil.log("---------- CYCLE Finished in : " + (System.currentTimeMillis() - startTime));
                return true; // OK to cycle again if there is time
            }
        }
        teamUtil.log("GRAB FAILED IN AUTO");
        intake.autoRetractAllAndUnload(true, 3000);
        return false; // no more cycling
    }


    public void sampleAutoV4(int[] position){
        boolean failed = false;
        int i = 0;
        long startTime = System.currentTimeMillis();
        drive.setRobotPosition(0,0,0);
        intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        deliver4Samples();
        teamUtil.log("---------- DELIVERED 4 in : " + (System.currentTimeMillis()-startTime));

        while(System.currentTimeMillis()-startTime<30000-Z0_CYCLE_GRAB_TIME && !teamUtil.theOpMode.isStopRequested()){
            if(!bucketCycleV2(startTime, position[i])){
                failed = true;
                teamUtil.log("Cycle Failed, bailing out");
                break;
            }
            i++;
        }
        teamUtil.log("No time left for cycling");
        if(failed){
            drive.stopMotors();
            outtake.setArmLevelOneAscent();
        }
        else if(!parked && System.currentTimeMillis()-startTime<30000-Z0_PARK_TIME){
            teamUtil.log("Parking");
            autoAscentPark();
            parked = true;
        }
        else{
            teamUtil.log("No time left to park");
            drive.driveMotorsHeadingsPower(315,315,1);
            teamUtil.pause(Z_FINAL_DROP_MOVE_TIME);
            drive.stopMotors();
        }
    }

    public void sampleAutoUnloadHighBucket(boolean fromSub){
        intake.autoRetractAllAndUnload(fromSub, 3000);
        output.outputHighBucket(3000); // TODO : add low bucket mode here to re-enable low bucket in teleop?
    }

    public void sampleAutoUnloadHighBucketNoWait(boolean fromSub) {
        teamUtil.log("Thread to sampleAutoUnloadHighBucket LAUNCHED");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    sampleAutoUnloadHighBucket(fromSub);
                }
            });
            thread.start();
    }


    public static long A0a_RELOAD_TIME = 500;

    public void autoGoToLoad(long timeout){
        long timeOutTime = System.currentTimeMillis()+timeout;
        output.bucket.setPosition(Output.BUCKET_RELOAD);
        teamUtil.pause(A0a_RELOAD_TIME);
        output.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        output.lift.setPower(-1);
        while(output.lift.getCurrentPosition()>Output.LIFT_DOWN&&teamUtil.keepGoing(timeOutTime)){
            teamUtil.pause(10);
        }
        output.lift.setPower(0);
    }

    public void autoGoToLoadNoWait(long timeout) {
        teamUtil.log("Thread to autoGoToLoadNoWait LAUNCHED");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    autoGoToLoad(timeout);
                }
            });
            thread.start();

    }

    public static int LIFT_THRESHOLD = 200;
    public void WaitForLiftReadyToUnloadSampleHighBucket(long timeout){
        teamUtil.log("WaitForLiftReadyToUnloadSampleHighBucket");
        long startTime = System.currentTimeMillis();
        long timeOutTime = System.currentTimeMillis()+timeout;
        while(output.lift.getCurrentPosition()<(Output.LIFT_TOP_BUCKET- LIFT_THRESHOLD)&&teamUtil.keepGoing(timeOutTime)){
            teamUtil.pause(4);
        }
        if (System.currentTimeMillis()>=timeOutTime) {
            teamUtil.log("WaitForLiftReadyToUnloadSampleHighBucket - TIMED OUT!");
        }
        teamUtil.log("WaitForLiftReadyToUnloadSampleHighBucket Waited for: " + (System.currentTimeMillis()-startTime));
    }

    public void outtakeUpAndGoToHighBucket(){
        outtake.outtakeRest();
        while(outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_OUTPUT_CLEAR && !teamUtil.theOpMode.isStopRequested()){
            teamUtil.pause(10);
        }
        output.outputHighBucket(3000);
    }

    public void outtakeUpandGoToHighBucketNoWait() {
        teamUtil.log("Thread to outtakeUpandGoToHighBucket LAUNCHED");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                outtakeUpAndGoToHighBucket();
            }
        });
        thread.start();

    }



/* OLD Code

    public static float B00_MAX_POWER = 1f;
    public static float B00_SUB_POWER = .4f;
    public static float B00_SUB_HOLD_POWER = .2f;
    public static float B00_ROT_ADJUST = 2f;

    public static int B01_SUB_X = 900;
    public static int B01_SUB_INITIAL_HEADING = 340;
    public static int B01_SUB_Y = -250;
    public static int B01_SUB_HEADING = 270;
    public static int B01_SUB_BRAKE_PAUSE = 200;
    public static int B01_SUB_STALL_PAUSE = 250;
    public static int B01_EXTENDER_POS = 250;
    public static int B01_SLIDER_POS = (int) AxonSlider.SLIDER_READY;

    public void bucketToSub(boolean launchIntake) {
        // chill out rotational adjust for these movements
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = BasicDrive.ROTATION_ADJUST_FACTOR_POWER/B00_ROT_ADJUST;

        // Move to sub
        drive.moveToX(B00_MAX_POWER, B01_SUB_X, B01_SUB_INITIAL_HEADING, B01_SUB_INITIAL_HEADING);
        if(launchIntake) AutoReadyToSeekNoWait(B01_SLIDER_POS, B01_EXTENDER_POS);
        drive.moveToY(B00_SUB_POWER, B01_SUB_Y, B01_SUB_HEADING, B01_SUB_HEADING);
        drive.stopMotors(); // fast braking
        teamUtil.pause(B01_SUB_BRAKE_PAUSE);

        // Use motors to hold robot against barrier while intaking
        drive.driveMotorsHeadingsFRPower(B01_SUB_HEADING, B01_SUB_HEADING, B00_SUB_HOLD_POWER);
        teamUtil.pause(B01_SUB_STALL_PAUSE); // give it a little time to get motionless

        // Restore rotational coefficient
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = BasicDrive.ROTATION_ADJUST_FACTOR_POWER*B00_ROT_ADJUST;
    }

    public static int B02_SUB_BACKUP_Y = -250;
    public static int B02_SUB_BACKUP_HEADING = 90;
    public static int B02_SUB_BASKET_X1 = 570; //maybe use 550
    public static int B02_SUB_BASKET_HEADING1 = 176;//was 160
    public static int B02_SUB_BASKET_HEADING2 = 315;
    public static int B02_BASKET_BRAKE_PAUSE = 200;
    public static int B02_MAX_SPEED_NEAR_BUCKET = 2500;
    public static int B02_END_SPEED_NEAR_BUCKET = 400;
    public static int B02_CYCLE_BUCKET_STRAIGHT = 90;
    public static int B02_CYCLE_BUCKET_STRAFE = 500;
    public static int B02_CYCLE_BUCKET_HEADING = 315;

    public void subToBucket() {
        // chill out rotational adjust for these movements
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = BasicDrive.ROTATION_ADJUST_FACTOR_POWER/B00_ROT_ADJUST;


        drive.moveToY(B00_MAX_POWER, B02_SUB_BACKUP_Y, B02_SUB_BACKUP_HEADING, B01_SUB_HEADING);
        drive.moveToX(B00_MAX_POWER, B02_SUB_BASKET_X1, B02_SUB_BASKET_HEADING1, (int) drive.adjustAngle(B02_SUB_BASKET_HEADING2));
        drive.stopMotors(); // fast braking
        teamUtil.pause(B02_BASKET_BRAKE_PAUSE);

        // Restore rotational coefficient for last movement to make sure we are aligned on bucket
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = BasicDrive.ROTATION_ADJUST_FACTOR_POWER*B00_ROT_ADJUST;
        drive.moveTo(B02_MAX_SPEED_NEAR_BUCKET,B02_CYCLE_BUCKET_STRAFE,B02_CYCLE_BUCKET_STRAIGHT,B02_CYCLE_BUCKET_HEADING,B02_END_SPEED_NEAR_BUCKET,null,0, false, 5000);
        drive.setMotorsActiveBrake();

    }

    public void bucketToSubV2 (int launchIntake) {
        // chill out rotational adjust for these movements
        float savedRotationAdjust = BasicDrive.ROTATION_ADJUST_FACTOR_POWER;
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = C00_ROTATION_ADJUST_FACTOR_POWER;

        // Move to sub
        drive.moveToX(C00_MAX_POWER, C01_SUB_X1, C02_BASKET_TO_SUB_HEADING, C02_BASKET_TO_SUB_HEADING);
        drive.moveToX(C00_MAX_POWER, C01_SUB_X, C02_BASKET_TO_SUB_HEADING, C01_SUB_HEADING);
        if(launchIntake>0) AutoReadyToSeekNoWait(C01_SLIDER_POS+((launchIntake-2)*S00_SLIDER_CYCLE_ADJUST), C01_EXTENDER_POS);
        drive.moveToY(C00_SUB_POWER, C01_SUB_Y, C01_SUB_HEADING, C01_SUB_HEADING);
        reverseUntilForwardMotionStoppedSample(BasicDrive.MAX_VELOCITY,1000); // hit the barrier while slowing down
        alignToSubmersibleSample(C00_SUB_ALIGN_POWER_1,C00_SUB_ALIGN_POWER_2,1500); // stop the bounce and the align to barrier
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        teamUtil.pause(C01_SUB_STALL_PAUSE); // give it a little more time to get motionless


        // Restore rotational coefficient
        BasicDrive.ROTATION_ADJUST_FACTOR_POWER = savedRotationAdjust;

    }

    public void bucketCycle(){
        bucketToSub(true);
        boolean grabbedSample = intake.autoGoToSampleAndGrabV3(false,false,true,3000);
        sampleAutoUnloadHighBucketNoWait(true);
        subToBucket();
        liftAtTop(2000);
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        teamUtil.pause(A03_DROP_TIME);
        autoGoToLoadNoWait(3000);
    }

    public void sampleAutoV3 () {
        long startTime = System.currentTimeMillis();
        drive.setRobotPosition(0,0,0);
        intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        deliver4Samples();
        if(System.currentTimeMillis()-startTime<Z0_TIME_LEFT_FOR_CYCLE){
            bucketCycle();
            if(System.currentTimeMillis()-startTime<25000){
                autoAscentPark();
            }
            else{
                teamUtil.log("No time left to park or cycle");
            }
        }
        else{
            teamUtil.log("No Time For Cycling");
        }

    }


    public boolean specimenCycleV3(int cycle, int cycleYTarget, boolean grabSample, boolean chillPickup, boolean getNextSpecimen){
        long startTime = System.currentTimeMillis();
        outtake.outakearm.setPosition(Outtake.ARM_UP);

        BasicDrive.MIN_STRAFE_START_VELOCITY = 2000;
        BasicDrive.MIN_START_VELOCITY = 1000;
        //drive.lastVelocity = BasicDrive.MAX_VELOCITY; //come off wall fast

        //Moves robot from the observation zone to the middle of the field in front of place position account for some strafe drift
        if (grabSample) {
            // Take a wider path if we are extending intake, turn motors on to hold robot against submersible
        } else {
            float strafeMaxDeclination = drive.STRAFE_MAX_DECLINATION;
            BasicDrive.STRAFE_MAX_DECLINATION = F18b_ADJUSTED_MAX_DECLINATION;
            drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, cycleYTarget - F0a_FAST_STRAFE_ADJUST, F19_CYCLE_MIDFIELD_X, 0, F08_TRANSITION_VELOCITY_FAST,
                    new BasicDrive.ActionCallback() {
                        @Override
                        public void action() {
                            outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
                        }
                    }, cycle == 2? F22b_CYCLE1_WRIST_CALLBACK:F22_CYCLE_WRIST_CALLBACK, 5000);
            BasicDrive.STRAFE_MAX_DECLINATION = strafeMaxDeclination;
            outtake.deployArm();

            if(cycle>=3){
                drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F21b_CYCLE_PLACE_SAMPLE_X, cycleYTarget, 0, BasicDrive.MAX_VELOCITY, false, null, 0, 4000);

                drive.driveMotorsHeadingsFR(180,0,BasicDrive.MAX_VELOCITY);
                teamUtil.pause(F23b_CYCLE_REVERSE_PLACE_SPECIMEN_PAUSE); // give it time to decelerate
            }
            else{
                drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F21_CYCLE_PLACE_SAMPLE_X, cycleYTarget, 0, BasicDrive.MAX_VELOCITY, false, null, 0, 4000);

                drive.stopMotors(); // TODO: Reconsider this, maybe reverse
                teamUtil.pause(F23_CYCLE_PLACE_SPECIMEN_PAUSE); // give it time to decelerate and coast to submersable
            }
        }

        boolean grabbedSample = false;
        if (grabSample) {
            if(teamUtil.alliance == RED){
                intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
            }
            else{
                intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
            }
            grabbedSample=intake.goToSampleAndGrabV3(false, false,true);
            autoRetractAndUnloadNoWait(grabbedSample);
            intake.lightsOff();
        }

        if(getNextSpecimen) {
            if (grabbedSample) {
                // go to unload position for next grab
            } else {
                if (chillPickup) {
                    // Old (slower) movements to make it easier on human player
                    //moves robot out of the way of the submersible
                    drive.lastVelocity = BasicDrive.MAX_VELOCITY; // back up fast
                    drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,F24_CYCLE_BACKUP_X,F02_PLACE_SPECIMEN_Y,0,F08_TRANSITION_VELOCITY_FAST, false, null,0,4000);
                    outtake.outtakeGrab();

                    //moves robot into position to drive forward to grab next specimen
                    drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,F25_CYCLE_PICKUP_Y+F0a_SLOW_STRAFE_ADJUST,F26_CYCLE_PREPARE_FOR_PICKUP_X,0, F03_TRANSITION_VELOCITY_CHILL,null,0,4000);

                    //moves robot to wall for grab
                    drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,F26a_CYCLE_PICKUP_X,F25_CYCLE_PICKUP_Y,0,F27_CYCLE_PICKUP_VELOCITY, false, null,0,4000);
                    teamUtil.pause(F28_CYCLE_PICKUP_PAUSE);



                } else {
                    //moves robot out of the way of the submersible
                    drive.lastVelocity = BasicDrive.MAX_VELOCITY; // back up fast
                    drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F24_CYCLE_BACKUP_X, F02_PLACE_SPECIMEN_Y, 0, F08_TRANSITION_VELOCITY_FAST, false, null, 0, 4000);
                    outtake.outtakeGrab();

                    //moves robot into position to drive forward to grab next specimen
                    drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, grabSample ? F32_CYCLE_PICKUP_Y_SPECIAL : F25_CYCLE_PICKUP_Y + F0a_FAST_STRAFE_ADJUST, F26_CYCLE_PREPARE_FOR_PICKUP_X, 0, F08_TRANSITION_VELOCITY_FAST, null, 0, 4000);

                    //moves robot to wall for grab
                    if (grabSample) dropSampleOutBackWithFlipperResetNoWait();
                    drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F26a_CYCLE_PICKUP_X, F25_CYCLE_PICKUP_Y, 0, F27_CYCLE_PICKUP_VELOCITY, false, null, 0, 4000);
                    teamUtil.pause(F28_CYCLE_PICKUP_PAUSE);
                }
            }
        }
        BasicDrive.MIN_STRAFE_START_VELOCITY = 500;
        BasicDrive.MIN_START_VELOCITY = 300;

        long elapsedTime = System.currentTimeMillis()-startTime;
        teamUtil.log("Cycle Time: "+elapsedTime);
        return true;
    }


    public boolean autoV3Specimen(int cycles){
        teamUtil.log("Running Specimen Auto V3.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));
        drive.setRobotPosition(0,0,0);
        specimenCollectBlocksV2();
        for(int i = 1; i<=cycles;i++){
            teamUtil.log("Auto V3 Specimen Cycle Number: " + i);
            specimenCycleV2(i, AUTO_INTAKE_SAMPLE&&i==1,i<cycles);
        }
        drive.stopMotors();
        //TODO make it so AUTO_INTAKE_SPECIMEN is an option when initializing
        return true;
    }

       public boolean specimenCycleV2(int cycle, boolean grabSample, boolean getNextSpecimen){
        long startTime = System.currentTimeMillis();
        outtake.outakearm.setPosition(Outtake.ARM_UP);

        BasicDrive.MIN_STRAFE_START_VELOCITY = 2000;
        BasicDrive.MIN_START_VELOCITY = 1000;
        drive.lastVelocity = BasicDrive.MAX_VELOCITY; //come off wall fast

        //Moves robot from the observation zone to the middle of the field in front of place position account for some strafe drift
        float strafeMaxDeclination = drive.STRAFE_MAX_DECLINATION;
        BasicDrive.STRAFE_MAX_DECLINATION = F18b_ADJUSTED_MAX_DECLINATION;
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, F33_6_CYCLE_Y_PLACEMENTS[cycle-1]-F0a_FAST_STRAFE_ADJUST,F19_CYCLE_MIDFIELD_X,0,F08_TRANSITION_VELOCITY_FAST,
                new BasicDrive.ActionCallback() {
                    @Override
                    public void action() {
                        outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
                        if (grabSample) AutoReadyToSeekNoWait(nextSliderPos, nextExtenderPos); // move intake out for the grab if needed
                    }
                },F22_CYCLE_WRIST_CALLBACK,5000);
        BasicDrive.STRAFE_MAX_DECLINATION = strafeMaxDeclination;
        outtake.deployArm();

        //moves robot from the middle of the field to scoring the specimen
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,F21_CYCLE_PLACE_SAMPLE_X, F33_6_CYCLE_Y_PLACEMENTS[cycle-1],0,BasicDrive.MAX_VELOCITY, false, null,0,4000);
        drive.stopMotors();
        teamUtil.pause(F23_CYCLE_PLACE_SPECIMEN_PAUSE); // give it time to decelerate and coast to submersable

        boolean grabbedSample = false;
        if (grabSample) {
            if(teamUtil.alliance == RED){
                intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
            }
            else{
                intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
            }
            grabbedSample=intake.goToSampleAndGrabV3(false, false,true);
            autoRetractAndUnloadNoWait(grabbedSample);
            intake.lightsOff();
        }

        if(getNextSpecimen) {
            //moves robot out of the way of the submersible
            drive.lastVelocity = BasicDrive.MAX_VELOCITY; // back up fast
            drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F24_CYCLE_BACKUP_X, F02_PLACE_SPECIMEN_Y, 0, F08_TRANSITION_VELOCITY_FAST, false, null, 0, 4000);
            outtake.outtakeGrab();

            //moves robot into position to drive forward to grab next specimen
            drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, grabSample ? F32_CYCLE_PICKUP_Y_SPECIAL : F25_CYCLE_PICKUP_Y + F0a_FAST_STRAFE_ADJUST, F26_CYCLE_PREPARE_FOR_PICKUP_X, 0, F08_TRANSITION_VELOCITY_FAST, null, 0, 4000);

            //moves robot to wall for grab
            if (grabSample) dropSampleOutBackWithFlipperResetNoWait();
            drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F26a_CYCLE_PICKUP_X, F25_CYCLE_PICKUP_Y, 0, F27_CYCLE_PICKUP_VELOCITY, false, null, 0, 4000);
            teamUtil.pause(F28_CYCLE_PICKUP_PAUSE);
            BasicDrive.MIN_STRAFE_START_VELOCITY = 500;
            BasicDrive.MIN_START_VELOCITY = 300;
        }

        long elapsedTime = System.currentTimeMillis()-startTime;
        teamUtil.log("Cycle Time: "+elapsedTime);
        return true;
    }

    public void hangPhase2NoWait(){

        teamUtil.log("hangPhase1NoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                hangPhase2();
            }
        });
        thread.start();

    }

    public void hangPhase2(){
        hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hang.engageHang();

       //  No real need to pull in slack at this point if strings are in the right spot
        teamUtil.pause(Hang.HANG_PHASE_2_SLACK_PAUSE);
        hang.hang_Left.setTargetPosition(Hang.SLACK_LEVEL);
        hang.hang_Right.setTargetPosition(Hang.SLACK_LEVEL);
        hang.hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Left.setVelocity(Hang.HANG_VELOCITY);
        hang.hang_Right.setVelocity(Hang.HANG_VELOCITY);

        teamUtil.pause(Hang.HANG_PHASE_2_ENGAGE_PAUSE); // don't put hooks on bar until we are off of ground
        output.lift.setVelocity(Robot.PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_AT_BAR);
        teamUtil.pause(Hang.HANG_PHASE_2_PLACE_PAUSE); // currently zero...there is enough slack now to where we can place the hooks and start pulling string at the same time
        hang.hang_Left.setTargetPosition(Hang.AUTO_LIFT_LEVEL);
        hang.hang_Right.setTargetPosition(Hang.AUTO_LIFT_LEVEL);
        hang.hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Left.setVelocity(Hang.HANG_VELOCITY);
        hang.hang_Right.setVelocity(Hang.HANG_VELOCITY);
        teamUtil.log("Both Joystick Drive Booleans HANGINGL and HANGINGR set true in hang Phase 2");
    hang.hangingL = true; hang.hangingR = true;// fake out control code to let it go up automatically until someone touches the joystick
}

public void hangPhase2V2(){
    long timeOutTime = System.currentTimeMillis() + 8000;
    hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    hang.engageHang();
    teamUtil.pause(Hang.HANG_PHASE_2_ENGAGE_PAUSE); // don't put hooks on bar until we are off of ground
    output.lift.setVelocity(Robot.PLACE_HOOKS_VELOCITY);
    output.lift.setTargetPosition(Output.LIFT_AT_BAR);
    hang.hang_Left.setTargetPosition(Hang.AUTO_LIFT_LEVEL);
    hang.hang_Right.setTargetPosition(Hang.AUTO_LIFT_LEVEL);
    hang.hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hang.hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hang.hang_Left.setVelocity(Hang.HANG_VELOCITY);
    hang.hang_Right.setVelocity(Hang.HANG_VELOCITY);

    teamUtil.pause(Hang.HANG_STARTUP_SPINDLE_PAUSE);
    int lastLeft = hang.hang_Left.getCurrentPosition();
    int lastRight = hang.hang_Right.getCurrentPosition();
    boolean leftTensioned = false;
    boolean rightTensioned = false;
    while ((!leftTensioned || !rightTensioned) && teamUtil.keepGoing(timeOutTime)) {
        dropLiftWhenNeeded();
        //teamUtil.log("Hangleft: " + hang.hang_Left.getCurrentPosition()+ " Hangright: "+ hang.hang_Right.getCurrentPosition());
        teamUtil.pause(100);
        if (!leftTensioned && hang.hang_Left.getCurrentPosition() - lastLeft < Hang.HANG_TENSION_THRESHOLD) {
            leftTensioned = true;
            hang.hang_Left.setTargetPosition(hang.hang_Left.getCurrentPosition());
            teamUtil.log("Left String Tensioned ");
        }
        if (!rightTensioned && hang.hang_Right.getCurrentPosition() - lastRight < Hang.HANG_TENSION_THRESHOLD) {
            rightTensioned = true;
            hang.hang_Right.setTargetPosition(hang.hang_Right.getCurrentPosition());
            teamUtil.log("Right String Tensioned ");
        }
        lastLeft = hang.hang_Left.getCurrentPosition();
        lastRight = hang.hang_Right.getCurrentPosition();
    }
    if (System.currentTimeMillis() > timeOutTime) {
        teamUtil.log("Hang Phase 2 V2 Timed Out!");
        return;
    }
    hang.hang_Right.setTargetPosition(hang.hang_Right.getCurrentPosition()+Hang.HANG_LEVEL_3);
    teamUtil.pause(Hang.HANG_PAUSE_FOR_LEFT);
    hang.hang_Left.setTargetPosition(hang.hang_Left.getCurrentPosition()+Hang.HANG_LEVEL_3);


    teamUtil.log("Both Joystick Drive Booleans HANGINGL and HANGINGR set true in hang Phase 2");
    hang.hangingL = true; hang.hangingR = true;// fake out control code to let it go up automatically until someone touches the joystick
}
    static public int A01_PLACE_SPECIMEN_X = 732;
    public static int A02_PLACE_SPECIMEN_Y = -50;
    static public int A03_MOVE_TO_SAMPLE_Y = 873;
    static public int A04_MOVE_TO_SAMPLE_X = 603;
    static public int A05_MOVE_TO_BUCKET_Y = 1084;
    static public int A06_MOVE_TO_BUCKET_X = 119;
    static public int A07_ROBOTHEADING_TO_TURN_TO_BUCKET = 315;
    static public int A08_MOVE_TO_SAMPLE_Y = 1145;
    static public int A09_MOVE_TO_SAMPLE_X = 603;
    static public int A10_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_X = A06_MOVE_TO_BUCKET_X+50;
    static public int A11_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_Y = A05_MOVE_TO_BUCKET_Y-50;
    static public float A12_SPECIMEN_MOTOR_POWER = .3f;
    static public int A13_SPECIMEN_END_VELOCITY = 500;
    static public long A14_SPECIMEN_PAUSE = 250;
    static public int A15_TRUSS_MOVEMENT_X = 1333;
    static public int A16_TRUSS_MOVEMENT_Y = 500;
    static public int A17_TRUSS_MOVEMENT_X = 1333;
    static public int A18_TRUSS_MOVEMENT_Y = 236;
    static public int A19_GO_TO_TRUSS_END_VELOCITY = 500;
    static public int A20_BACKOUT_FROM_TRUSS_END_VELOCITY = 500;
    static public float A21_LEVEL_ONE_ASCENT_DRIVE_POWER = 0.3f;
    public static int A22_LEVEL_ONE_ASCENT_DRIVE_POWER_TIME = 100;


    static public int B01_PLACE_SPECIMEN_X = 760;
    public static int B02_PLACE_SPECIMEN_Y = 120;
    public static int B02a_TRANSITION_VELOCITY = 500;
    static public int B03_END_VELOCITY_SPECIMEN = 200;
    static public float B04_SPECIMEN_MOTOR_POWER = 0.3f;
    static public int B05_SPECIMEN_PAUSE = 250;
    static public int B06_END_VELOCITY_SPECIMEN_BACKUP = 500;
    public static int B07_GO_TO_SAMPLE_X = 600;
    public static int B08_GO_TO_SAMPLE_Y = -1003;


    public static int B09_END_VELOCITY_SEEK_AFTER_BACKUP = 0;
    public static int B10_END_VELOCITY_SPECIMEN = 500;
    public static int B11_WALL_SPECIMEN_X = 100;
    public static int B12_WALL_SPECIMEN_Y = -900;
    public static float B13_SPECIMENDROP_MOTOR_POWER = 0.1f;


    public static int C0a_FAST_STRAFE_ADJUST = 250;
    public static int C0a_FAST_STRAIGHT_ADJUST1 = 100;
    public static int C0a_FAST_STRAIGHT_ADJUST2 = 200;
    public static int C0a_FAST_REVERSE_ADJUST = 50;
    public static int C0a_SLOW_STRAFE_ADJUST = 50;
    static public int C01_PLACE_SPECIMEN_X = 760;
    public static int C02_PLACE_SPECIMEN_Y = 120;
    public static int C03_TRANSITION_VELOCITY_CHILL = 500;
    static public int C04_END_VELOCITY_SPECIMEN = 600;
    static public int C05_SPECIMEN_PAUSE = 250;
    static public int C06_CLEAR_SUB_X = 600;
    static public int C07_CLEAR_SUB_Y = -660;
    public static int C08_TRANSITION_VELOCITY_FAST = 1500;
    static public int C09_CLEAR_SAMPLE_X = 1100;
    static public int C10_SAMPLE_1_Y = -900;
    static public int C10_SAMPLE_Y_ADJUST = 50;
    static public int C11_DROP_SAMPLE_X = 250;
    public static int C12_TRANSITION_VELOCITY_REVERSE = 1000;
    static public int C13_SAMPLE_2_Y = C10_SAMPLE_1_Y-180;
    static public int C14_SAMPLE_3_Y = C13_SAMPLE_2_Y-170;
    static public int C15_BACK_OUT_OBSERVATION_ZONE = 550;

    public static int D0a_FAST_STRAFE_ADJUST = 250;
    public static int D0a_FAST_STRAIGHT_ADJUST1 = 100;
    public static int D0a_FAST_STRAIGHT_ADJUST2 = 200;
    public static int D0a_FAST_REVERSE_ADJUST = 50;
    static public int D00_WALL_TO_MIDFIELD_X = 199;
    static public int D01_WALL_TO_MIDFIELD_Y = -50;
    public static int D02_TRANSITION_VELOCITY_FAST = 2000;
    static public int D03_MIDFIELD_TO_CHAMBER_X = 600;
    static public int D04_MIDFIELD_TO_CHAMBER_Y = 100;
    static public int D05_CHAMBER_TO_MIDFIELD_X = 657;
    static public int D06_CHAMBER_TO_MIDFIELD_Y = 47;
    static public int D07_SPECIMEN_PAUSE = 250;
    static public int D08_PREPARE_FOR_PICKUP_X=370;
    static public int D09_PREPARE_FOR_PICKUP_Y=-432;
    public static int D10_TRANSITION_VELOCITY_SLOW = 750;
    static public int D11_PICKUP_Y = -536;
    static public int D12_PICKUP_X = 75;
    static public int D13_GRAB_SPECIMEN_END_VELOCITY = 600;
    static public int D14_SPECIMEN_GRAB_TIME = 250;
    static public int D15_CYCLE_SPECIMEN_ADJUSTMENT = 50;
    static public int D16_WRIST_CALLBACK = -600;

    static public int E0_FIRST_PICKUP_Y = -980;
    static public int E0_FIRST_PICKUP_X = 350;
    static public int E1_EXTENDER_SPECIMEN_PICKUP = 504;
    static public long E2_EXTENDER_SPECIMEN_PAUSE_1 = 500;
    static public double E3_SLIDER_THIRD_GRAB = -1560;
    static public int E4_1STHASH_TO_2NDHASH_MOVEMENT_Y = -250;
    static public int E5_2NDHASH_TO_3RDHASH_MOVEMENT_Y = -100;
    public boolean autoV1Bucket(int blocks, boolean ascent){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));

        drive.setRobotPosition(0,0,0);
        //Get intake and output into positions

        outtake.deployArm();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);

        // Get close to submersible then mechanically align
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A01_PLACE_SPECIMEN_X,A02_PLACE_SPECIMEN_Y,0,A13_SPECIMEN_END_VELOCITY, false, null,0,4000);
        drive.setMotorPower(A12_SPECIMEN_MOTOR_POWER);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLETHEARTBEAT);
        teamUtil.pause(A14_SPECIMEN_PAUSE);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        //if (!placeSpecimen(2000)) {
        //    teamUtil.log("FAILED to place specimen.  Giving up");
        //    return false;
        //}



        drive.stopMotors();

    // Move over to first yellow sample
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A04_MOVE_TO_SAMPLE_X,0,0,500, false, null,0,4000);
        outtake.outtakeRest();
        intake.goToSeekNoWait(2000);
        drive.moveTo(BasicDrive.MAX_VELOCITY,A03_MOVE_TO_SAMPLE_Y,A04_MOVE_TO_SAMPLE_X, 0,0, null,0, 5000);
        intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);


        //if(!intake.goToSampleAndGrab(5000)){
        //    teamUtil.log("FAILED to intake sample.  Giving up");
        //    return false;



    dropSampleInHighBucket(1);
        if(blocks==1){
        teamUtil.log("Stopped After 1 Block");

        if(ascent){
            outtake.setArmLevelOneAscent();
            drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A19_GO_TO_TRUSS_END_VELOCITY,null,0,4000);
            drive.moveTo(BasicDrive.MAX_VELOCITY, A18_TRUSS_MOVEMENT_Y,A17_TRUSS_MOVEMENT_X,270,0,null,0,4000);
            drive.setMotorPower(A21_LEVEL_ONE_ASCENT_DRIVE_POWER);
            teamUtil.pause(A22_LEVEL_ONE_ASCENT_DRIVE_POWER_TIME);
            drive.stopMotors();

        }
        else{
            drive.stopMotors();
        }

        return true;
    }
        intake.goToSeekNoWait(2000);
        drive.moveTo(BasicDrive.MAX_VELOCITY, A08_MOVE_TO_SAMPLE_Y, A04_MOVE_TO_SAMPLE_X,0,0, null,0, 5000);

        //if(!intake.goToSampleAndGrab(5000)){
        //    teamUtil.log("FAILED to intake sample.  Giving up");
        //    return false;
        //}



    dropSampleInHighBucket(2);
        if(blocks==2){
        teamUtil.log("Stopped After 2 Blocks");
        if(ascent){
            outtake.setArmLevelOneAscent();
            drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A19_GO_TO_TRUSS_END_VELOCITY,null,0,4000);
            drive.moveTo(BasicDrive.MAX_VELOCITY, A18_TRUSS_MOVEMENT_Y,A17_TRUSS_MOVEMENT_X,270,0,null,0,4000);
            drive.setMotorPower(A21_LEVEL_ONE_ASCENT_DRIVE_POWER);
            teamUtil.pause(A22_LEVEL_ONE_ASCENT_DRIVE_POWER_TIME);
            drive.stopMotors();


        }
        else{
            drive.stopMotors();
        }

        return true;
    }
        drive.stopMotors();

        drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A19_GO_TO_TRUSS_END_VELOCITY,null,0,4000);
        intake.goToSeekNoWait(2000);

        drive.moveTo(BasicDrive.MAX_VELOCITY, A18_TRUSS_MOVEMENT_Y,A17_TRUSS_MOVEMENT_X,270,0,null,0,4000);

    //if(!intake.goToSampleAndGrab(5000)){
    //    teamUtil.log("FAILED to intake sample.  Giving up");
    //    return false;



    sampleInBucketAndDeployNoWait();
        drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A20_BACKOUT_FROM_TRUSS_END_VELOCITY,null,0,4000);
    dropSampleInHighBucket(3);
        if(blocks==3){
        teamUtil.log("Stopped After 3 Blocks");

        return true;
    }


        return true;
}

    public boolean specimenArmsWORKINPROGRESS(){
        drive.setRobotPosition(0,0,0);
        //Move to pickup spot 1 (launch extenders during movement as action)
        intake.goToSeek(1000);
        outtake.outtakeGrab();
        intake.lightsOnandOff(Intake.WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,true);
        intake.startCVPipeline();
        intake.extendersToPositionMaxVelo(E1_EXTENDER_SPECIMEN_PICKUP,3000);
        //TODO take above out



        teamUtil.pause(E2_EXTENDER_SPECIMEN_PAUSE_1);
        if(teamUtil.alliance == RED) intake.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
        else intake.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
        intake.goToSampleV5(1000);
        intake.flipToSampleAndGrab(1000);
        unloadNoWait(true, AxonSlider.SLIDER_READY);
        //move to spot 2 (with drive)
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,E4_1STHASH_TO_2NDHASH_MOVEMENT_Y,0,0,0,null,0,2000);
        if(autoUnloadNoWaitDone.get()) teamUtil.pause(E2_EXTENDER_SPECIMEN_PAUSE_1);
        while(!autoUnloadNoWaitDone.get()){
            teamUtil.pause(50);
        }
        //TODO Make sure drive constants are good
        intake.goToSampleV5(1000);
        intake.flipToSampleAndGrab(1000);
        unloadNoWait(true, E3_SLIDER_THIRD_GRAB);
        //move to spot 3 (with drive)
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,E4_1STHASH_TO_2NDHASH_MOVEMENT_Y+E5_2NDHASH_TO_3RDHASH_MOVEMENT_Y,0,0,0,null,0,2000);
        if(autoUnloadNoWaitDone.get()) teamUtil.pause(E2_EXTENDER_SPECIMEN_PAUSE_1);
        while(!autoUnloadNoWaitDone.get()){
            teamUtil.pause(50);
        }

        teamUtil.log("SpecimenPickUpAll has finished");
        return true;
    }

    public void unloadNoWait(boolean seekAgain, double strafeTarget){
        teamUtil.log("Launching Thread to specimenPickUpNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoUnloadNoWaitDone.set(false);
                intake.goToSafeRetract(2500);
                intake.extender.setTargetPositionTolerance(Intake.EXTENDER_TOLERANCE_RETRACT);
                //TODO: Coach: should we be setting a velocity here?
                intake.extendersToPositionMaxVelo(Intake.EXTENDER_UNLOAD,3000);
                intake.unload();
                teamUtil.pause(100);
                output.dropSampleOutBackNoWait();


                if(seekAgain){
                    intake.goToSeek(1000);
                    intake.axonSlider.runToEncoderPositionNoWait(strafeTarget,3000);
                    intake.extendersToPositionMaxVelo(E1_EXTENDER_SPECIMEN_PICKUP,3000);
                }
                autoUnloadNoWaitDone.set(true);

            }
        });
        thread.start();

    }





    public void sampleInBucketAndDeploy(){
        intake.unloadV2(false);
        output.outputHighBucket(3000);
    }

    public void sampleInBucketAndDeployNoWait(){
        teamUtil.log("Launching Thread to outputLoadNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                sampleInBucketAndDeploy();
            }
        });
        thread.start();
    }


    public void dropSampleInHighBucket(int cycle){
        //THIS ASSUMES THAT ROBOT WILL MOVE AFTER THIS METHOD IS CALLED
        if(cycle<3){
            sampleInBucketAndDeployNoWait();
        }

        drive.moveTo(BasicDrive.MAX_VELOCITY,A05_MOVE_TO_BUCKET_Y,A06_MOVE_TO_BUCKET_X,A07_ROBOTHEADING_TO_TURN_TO_BUCKET,0, null,0, 5000);
        while(output.lift.getCurrentPosition()<(Output.LIFT_TOP_BUCKET-10)){
            teamUtil.pause(50);
        }
        output.dropSampleOutBack();
        output.outputLoadNoWait(4000);
        drive.moveTo(BasicDrive.MAX_VELOCITY, A11_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_Y, A10_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_X, A07_ROBOTHEADING_TO_TURN_TO_BUCKET, 300,null,0, 2000);
    }


    public int fieldSide() { // helper method that returns heading out towards the field
        return teamUtil.alliance == RED ? 90 : 270;
    }

    public int driverSide() { // helper method that returns heading backs towards drivers
        return teamUtil.alliance == RED ? 270 : 90;
    }

    public int audienceSide() {
        return 180;
    }

    public int scoreSide() {
        return 0;
    }


    */








}

