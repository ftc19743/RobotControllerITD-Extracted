package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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



    public static boolean AUTO_INTAKE_SAMPLE = true;

    public static int F0a_FAST_STRAFE_ADJUST = 300;
    public static int F0a_FAST_STRAIGHT_ADJUST1 = 200;
    public static int F0a_FAST_STRAIGHT_ADJUST2 = 250;
    public static int F0a_FAST_REVERSE_ADJUST = 0;
    public static int F0a_SLOW_STRAFE_ADJUST = 50;
    public static int F02_PLACE_SPECIMEN_Y = 0; // was 130
    public static int F03_TRANSITION_VELOCITY_SLOW = 750;
    public static int F03_TRANSITION_VELOCITY_CHILL = 500;
    static public int F06_CLEAR_SUB_X = 600;
    static public int F07_CLEAR_SUB_Y = -800; // was -660
    public static int F08_TRANSITION_VELOCITY_FAST = 2000;
    static public int F09_CLEAR_SAMPLE_X = 1100;
    static public int F10_SAMPLE_1_Y = -980; // was -840
    static public int F10_SAMPLE_Y_ADJUST = 50;
    static public int F11_DROP_SAMPLE1_X = 600;
    public static int F12_REVERSE_BRAKING_PAUSE = 150;
    public static int F12_REVERSE_BRAKING_PAUSE2 = 200;
    static public int F13_SAMPLE_2_Y = -1220; // was -1080
    static public int F13a_DROP_SAMPLE2_X = 500;
    static public int F14_SAMPLE_3_Y = -1470; // was -1280 // also adjusted 50 for better wall contact
    static public int F14a_SAMPLE3_Y_ADJUST = 100; // adjusted 50 for better wall contact
    static public int F14a_DROP_SAMPLE3_X = 600;
    static public int F15_PICKUP_1_X = 150;
    static public int F15_PICKUP_1_Y = -1350; // was -1150
    static public int F16_PICKUP_1_VELOCITY = 600;
    static public int F17_PICKUP_1_PAUSE = 400;
    static public int F18_CYCLE_PLACE_SPECIMEN_1_Y = -70; // was 140
    static public int F18a_CYCLE_PLACE_SPECIMEN_Y = -40; //was 140
    static public float F18b_ADJUSTED_MAX_DECLINATION = 35;
    static public int F19_CYCLE_MIDFIELD_X = 500; // was 550 when we were trying for 6

    static public int F20_CYCLE_SPECIMEN_Y_ADJUSTMENT = 25;
    static public int F21_CYCLE_PLACE_SAMPLE_X = 700;
    static public int F21b_CYCLE_PLACE_SAMPLE_X = 690;
    static public int F22_CYCLE_WRIST_CALLBACK = -600;
    static public int F22b_CYCLE1_WRIST_CALLBACK = -800;
    static public int F23_CYCLE_PLACE_SPECIMEN_PAUSE = 200;
    static public int F23b_CYCLE_REVERSE_PLACE_SPECIMEN_PAUSE = 200;

    static public int F24_CYCLE_BACKUP_X = 700;
    static public int F25_CYCLE_PICKUP_Y = -690; //was 550
    static public int F26_CYCLE_PREPARE_FOR_PICKUP_X = 300;
    static public int F26a_CYCLE_PICKUP_X = 75;
    static public int F27_CYCLE_PICKUP_VELOCITY = 500;
    static public int F28_CYCLE_PICKUP_PAUSE = 200;
    static public long F29_AUTO_MOMENTUM_PAUSE = 200;
    static public long F32_CYCLE_PICKUP_Y_SPECIAL = -790; //was -650
    static public long DROP_SAMPLE_OUT_BACK_WITH_FLIPPER_RESET_1 = 500;
    static public long DROP_SAMPLE_OUT_BACK_WITH_FLIPPER_RESET_2 = 300;

    public AtomicBoolean autoUnloadNoWaitDone = new AtomicBoolean(false);









    static public boolean waitingForButtonPress = true;
    public static double OUTAKE_ARM_ENGAGE_VAL = 0;

    static public boolean AA_DEBUG_AUTO = false;
    //public Intake intake;
    //public Output output;
    //public Lift lift;
    //public PixelRelease releaser;
    //public Launcher launcher;

    public double a,b,c,d;

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

    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        outtake = new Outtake();
        intake = new Intake();
        output = new Output();
        hang = new Hang();
        teamUtil.robot = this;

    }

    public void initialize() {
        outtake.initalize();
        drive.initalize();
        output.initalize();
        intake.initialize();
        hang.initalize();
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
        hang.calibrate();

        outtake.firstCalibrate();
        output.calibrate();
        intake.calibrate();
        outtake.secondCalibrate();
    }

    public void resetRobot(){
        outtake.outtakeRest();
        teamUtil.pause(2000);
        intake.goToSafe();
        teamUtil.pause(2000);
        output.outputLoad(4000);
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

    public void goToSampleAndGrabAndLiftToBucket(boolean HighBucket){
        if(!intake.goToSampleAndGrabV3(true, true)){
            return;
        }
        if(HighBucket){
            output.outputHighBucket(3000);
        }
        else{
            output.outputLowBucket(3000);
        }
        intake.flipperGoToSafe(3000);
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

    public void AutoReadyToSeek(int sliderPos, int extenderPos) {
        // get flipper/wrist in position and turn lights on
        intake.goToSeekNoExtenders();
        // move extender and flipper to specified positions
        intake.axonSlider.runToEncoderPositionNoWait(sliderPos, true, 1500); // get slider going
        intake.extendersToPositionMaxVelo(extenderPos,1500);
    }

    public void AutoReadyToSeekNoWait(int sliderPos, int extenderPos) {
        teamUtil.log("Launching Thread to AutoReadyToSeek");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                AutoReadyToSeek(sliderPos, extenderPos);
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

    static public int F01_PLACE_SPECIMEN1_X = 580;
    static public int F01_PLACE_SPECIMEN1_Y = 0;
    static public int F01_SPECIMEN1_PAUSE1 = 200;
    static public int F01_SPECIMEN1_PAUSE2 = 200;
    static public int F01_SPECIMEN1_END_VELOCITY = 600;

    public void placeFirstSpecimen() {
        //Drive to the submersible while moving a bit to the left
        outtake.deployArm();
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F01_PLACE_SPECIMEN1_X, F01_PLACE_SPECIMEN1_Y,0,BasicDrive.MAX_VELOCITY, false, null,0,4000);
        drive.driveMotorsHeadingsFR(180,0,BasicDrive.MAX_VELOCITY);
        //drive.stopMotors(); // TODO: Try reverse motors instead
        teamUtil.pause(F01_SPECIMEN1_PAUSE1); // give it time to decelerate
        drive.driveMotorsHeadingsFR(0,0,F01_SPECIMEN1_END_VELOCITY);
        teamUtil.pause(F01_SPECIMEN1_PAUSE2); // give it time to click in
    }

    //Collects all blocks using arms
    public void specimenCollectBlocksV2() {
        drive.setRobotPosition(0,0,0);
        drive.setMotorsBrake();
        long startTime = System.currentTimeMillis();

        placeFirstSpecimen();
        teamUtil.log("First Specimen Dropped: "+ (System.currentTimeMillis()-startTime));

        //TODO take image here to adjust the slider/extender encoder target values we send the intake to on the first cycle?

        // Back up to clear sub
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F06_CLEAR_SUB_X, F02_PLACE_SPECIMEN_Y,0,F08_TRANSITION_VELOCITY_FAST, false, null,0,4000);
        outtake.outtakeGrab();
        // strafe over to clear sub on other side
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, F07_CLEAR_SUB_Y+F0a_FAST_STRAFE_ADJUST, F06_CLEAR_SUB_X, 0, F08_TRANSITION_VELOCITY_FAST,null, 0, 2000);

        // drive past samples
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F09_CLEAR_SAMPLE_X- F0a_FAST_STRAIGHT_ADJUST1, F07_CLEAR_SUB_Y,0,F08_TRANSITION_VELOCITY_FAST, false, null,0,4000);

        // strafe to sample 1
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, F10_SAMPLE_1_Y+F10_SAMPLE_Y_ADJUST, F09_CLEAR_SAMPLE_X, 0, F08_TRANSITION_VELOCITY_FAST,null, 0, 2000);

        // push first sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F11_DROP_SAMPLE1_X +F0a_FAST_REVERSE_ADJUST, F10_SAMPLE_1_Y,0,BasicDrive.MAX_VELOCITY, false, null,0,4000);
        //drive.stopMotors(); // faster to just reverse motors at full speed
        //teamUtil.pause(F12_REVERSE_BRAKING_PAUSE);
        // head back out to get second sample
        drive.lastVelocity = BasicDrive.MAX_VELOCITY; // make motors start at full power
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F09_CLEAR_SAMPLE_X- F0a_FAST_STRAIGHT_ADJUST2, F10_SAMPLE_1_Y,0,F08_TRANSITION_VELOCITY_FAST, false, null,0,4000);
        drive.strafeToTarget(270,0,F08_TRANSITION_VELOCITY_FAST, F13_SAMPLE_2_Y+F10_SAMPLE_Y_ADJUST,2000);

        // push second sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F13a_DROP_SAMPLE2_X +F0a_FAST_REVERSE_ADJUST, F13_SAMPLE_2_Y,0,BasicDrive.MAX_VELOCITY, false, null,0,4000);
        //drive.stopMotors(); // faster to just reverse motors at full speed
        //teamUtil.pause(F12_REVERSE_BRAKING_PAUSE);

        // head back out for 3rd sample
        drive.lastVelocity = BasicDrive.MAX_VELOCITY; // make motors start at full power
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F09_CLEAR_SAMPLE_X- F0a_FAST_STRAIGHT_ADJUST2, F13_SAMPLE_2_Y,0,F08_TRANSITION_VELOCITY_FAST,false,null,0,4000);
        drive.strafeToTarget(270,0,F08_TRANSITION_VELOCITY_FAST, F14_SAMPLE_3_Y+F14a_SAMPLE3_Y_ADJUST,2000);

        // push 3rd sample to observation zone and grab 2nd specimen
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F14a_DROP_SAMPLE3_X +F0a_FAST_REVERSE_ADJUST, F14_SAMPLE_3_Y,0,BasicDrive.MAX_VELOCITY,false,null,0,4000);
        drive.stopMotors(); // TODO: Speed up this part.  Consider reverse motors, coming into wall at an angle, faster velocities
        teamUtil.pause(F12_REVERSE_BRAKING_PAUSE2);
        // Move over a bit to ensure 3rd Sample doesn't block us on wall
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,F15_PICKUP_1_Y-F10_SAMPLE_Y_ADJUST, F11_DROP_SAMPLE1_X,0,F03_TRANSITION_VELOCITY_SLOW,null,0,4000);
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,F15_PICKUP_1_X,F15_PICKUP_1_Y,0,F16_PICKUP_1_VELOCITY, false, null,0,4000);
        teamUtil.pause(F17_PICKUP_1_PAUSE);
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
                    }, cycle == 1? F22b_CYCLE1_WRIST_CALLBACK:F22_CYCLE_WRIST_CALLBACK, 5000);
            BasicDrive.STRAFE_MAX_DECLINATION = strafeMaxDeclination;
            outtake.deployArm();

            if(cycle>=2){
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
            grabbedSample=intake.goToSampleAndGrabV3(false, false);
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

    static public int[] F33_6_CYCLE_Y_PLACEMENTS = {100, 80, 60, 40, 20};
    static public int[] F33_5_CYCLE_Y_PLACEMENTS = {0, 47, 94, 140};

    public void fiveSpecimenCycle() {
        specimenCycleV3(1, F33_5_CYCLE_Y_PLACEMENTS[0],false, true, true);
        specimenCycleV3(2, F33_5_CYCLE_Y_PLACEMENTS[1],false, true, true);
        specimenCycleV3(3, F33_5_CYCLE_Y_PLACEMENTS[2],false, true, true);
        specimenCycleV3(4, F33_5_CYCLE_Y_PLACEMENTS[3],false, true, true);
    }
    public void sixSpecimenCycle() {
        specimenCycleV3(1, F33_6_CYCLE_Y_PLACEMENTS[0], true, false, true);
        specimenCycleV3(2, F33_6_CYCLE_Y_PLACEMENTS[1], false, false, true);
        specimenCycleV3(3, F33_6_CYCLE_Y_PLACEMENTS[2], false, false, true);
        specimenCycleV3(4, F33_6_CYCLE_Y_PLACEMENTS[3], false, false, true);
        specimenCycleV3(5, F33_6_CYCLE_Y_PLACEMENTS[4], false, false, false);

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
            grabbedSample=intake.goToSampleAndGrabV3(false, false);
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

    public boolean autoV4Specimen(){
        teamUtil.log("Running Specimen Auto V4.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));
        drive.setRobotPosition(0,0,0);
        specimenCollectBlocksV2();
        fiveSpecimenCycle();
        drive.stopMotors();
        //TODO make it so AUTO_INTAKE_SPECIMEN is an option when initializing
        return true;
    }



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HANG
    public static int PICK_UP_HOOKS_PAUSE_1 = 450;
    public static int PICK_UP_HOOKS_PAUSE_2 = 300;
    public static int PICK_UP_HOOKS_PAUSE_3 = 250;
    public static int PICK_UP_HOOKS_PAUSE_4 = 500;
    public static int READY_TO_PLACE_HOOKS_PAUSE_1 = 250;
    public static int READY_TO_PLACE_HOOKS_VELOCITY = 1400;
    public static int PLACE_HOOKS_VELOCITY = 400;


    public void hangPhase1(){
        hang.extendHang();
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

    public void hangPhase2(){
        hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hang.engageHang();

        /*  No real need to pull in slack at this point if strings are in the right spot
        teamUtil.pause(Hang.HANG_PHASE_2_SLACK_PAUSE);
        hang.hang_Left.setTargetPosition(Hang.SLACK_LEVEL);
        hang.hang_Right.setTargetPosition(Hang.SLACK_LEVEL);
        hang.hang_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.hang_Left.setVelocity(Hang.HANG_VELOCITY);
        hang.hang_Right.setVelocity(Hang.HANG_VELOCITY);
*/
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
            teamUtil.log("Hangleft: " + hang.hang_Left.getCurrentPosition()+ " Hangright: "+ hang.hang_Right.getCurrentPosition());
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
    public void hangPhase2V3(){
        long timeOutTime = System.currentTimeMillis() + 8000;
        hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Assumes 33" of string out
        hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Assumes 39" of string out

        hang.engageHang();
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
            dropLiftWhenNeeded();
            teamUtil.log("Hangleft: " + hang.hang_Left.getCurrentPosition()+ " HangRight: "+ hang.hang_Right.getCurrentPosition());
            teamUtil.pause(50);
        }
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("Hang Phase 2 V3 Timed Out!");
            return;
        }

        // Head to the top
        teamUtil.log("Heading Up");
        hang.hang_Right.setTargetPosition(Hang.HANG_LEVEL_3_R);
        hang.hang_Left.setTargetPosition(Hang.HANG_LEVEL_3_L);

        teamUtil.log("Both Joystick Drive Booleans HANGINGL and HANGINGR set true in hang Phase 2");
        hang.hangingL = true; hang.hangingR = true;// fake out control code to let it go up automatically until someone touches the joystick
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

    public void dropLift() {
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_PRE_RELEASE);
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY); // Run to near bottom
        output.lift.setTargetPosition(Output.LIFT_DOWN);
        while (output.lift.getCurrentPosition() > Output.LIFT_DOWN+10) {
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
            hang.stowHang();
        }


    }

    boolean hookArmMoved = false;
    public void moveHookArmWhenNeeded() {
        if (hang.hang_Left.getCurrentPosition() > Hang.HOOK_ARM_MOVED_ON_WAY_UP && !hookArmMoved) {
            hookArmMoved = true;
            hang.deployHookGrabber();
        }

    }

    public void pickUpHooks(){
        intake.flipper.setPosition(Intake.FLIPPER_SAFE);
        outtake.outakearm.setPosition(Outtake.ARM_ENGAGE);
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY);
        hang.extendHang();

        output.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        output.lift.setTargetPosition(Output.LIFT_SAFE_FOR_HOOK_HOLDER);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_1);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_2);
        output.lift.setTargetPosition(Output.LIFT_PICKUP_FOR_HOOK_HOLDER);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_3);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_GRAB);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_4);

    }

    public void readyToPlaceHooks(){
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(READY_TO_PLACE_HOOKS_PAUSE_1);
        output.lift.setVelocity(READY_TO_PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_ABOVE_BAR);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_DEPLOY);
    }

    public void placeHooks(){
        output.lift.setVelocity(PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_ONTO_BAR);
        while (output.lift.getCurrentPosition() > Output.LIFT_ONTO_BAR+10) { // wait for hooks to be released
            teamUtil.pause(50);
        }
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY); // Run to near bottom
        output.lift.setTargetPosition(Output.LIFT_DOWN+30);
        while (output.lift.getCurrentPosition() > Output.LIFT_DOWN+40) {
            teamUtil.pause(50);
        }
        output.lift.setVelocity(0); // Turn off lift motor at bottom
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM); // rotate bucket to avoid string while tensioning

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

    public void placeHooksNoWait() {
        if (hang.hangMoving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to placeHooks while hang is moving--ignored");
            return;
        } else {
            teamUtil.log("Launching Thread to placeHooks");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    placeHooks();
                }
            });
            thread.start();
        }

    }




/* OLD Code

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
    public boolean placeSpecimen(long timeout) {
        teamUtil.log("Place Specimen");
        long timeoutTime = System.currentTimeMillis()+timeout;
        outtake.outakearm.setPosition(OUTAKE_ARM_ENGAGE_VAL);
        while(outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_ATTACH && teamUtil.keepGoing(timeoutTime)){
        }
        if (System.currentTimeMillis() >= timeoutTime) {
            teamUtil.log("Place Specimen -- TIMED OUT");
            return false;
        }
        teamUtil.log("Place Specimen -- Finished");
        return true;
    }

    public boolean specimenCycle(int cycles){
        outtake.outakearm.setPosition(Outtake.ARM_UP);

        BasicDrive.MIN_STRAFE_START_VELOCITY = 2000;
        BasicDrive.MIN_START_VELOCITY = 1000;
        //Moves robot from the observation zone to the middle of the field
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,D01_WALL_TO_MIDFIELD_Y,D00_WALL_TO_MIDFIELD_X,0,D02_TRANSITION_VELOCITY_FAST,
                new BasicDrive.ActionCallback() {
                    @Override
                    public void action() {
                        outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
                    }
                },D16_WRIST_CALLBACK,5000);
        outtake.deployArm();

        //moves robot from the middle of the field to scoring the specimen
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,D03_MIDFIELD_TO_CHAMBER_X,D04_MIDFIELD_TO_CHAMBER_Y+(D15_CYCLE_SPECIMEN_ADJUSTMENT*(cycles-1)),0,C04_END_VELOCITY_SPECIMEN,false,null,0,4000);
        teamUtil.pause(D07_SPECIMEN_PAUSE);

        //moves robot out of the way of the submersible
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,D05_CHAMBER_TO_MIDFIELD_X,D06_CHAMBER_TO_MIDFIELD_Y,0,D02_TRANSITION_VELOCITY_FAST,false,null,0,4000);
        outtake.outtakeGrab();

        //moves robot into postion to drive forward to grab next specimen
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,D09_PREPARE_FOR_PICKUP_Y,D08_PREPARE_FOR_PICKUP_X,0, D10_TRANSITION_VELOCITY_SLOW,null,0,4000);

        //moves robot to wall for grab
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,D12_PICKUP_X,D11_PICKUP_Y,0,D13_GRAB_SPECIMEN_END_VELOCITY,false,null,0,4000);
        teamUtil.pause(D14_SPECIMEN_GRAB_TIME);
        BasicDrive.MIN_STRAFE_START_VELOCITY = 500;
        BasicDrive.MIN_START_VELOCITY = 300;
        return true;
    }

    public void specimenCollectBlocks() {
        drive.setRobotPosition(0,0,0);
        long startTime = System.currentTimeMillis();
        //Drive to the submersible while moving a bit to the left
        outtake.deployArm();
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C01_PLACE_SPECIMEN_X, C02_PLACE_SPECIMEN_Y,0,C04_END_VELOCITY_SPECIMEN, false, null,0,4000);
        teamUtil.pause(C05_SPECIMEN_PAUSE); // give it time to click in
        teamUtil.log("First Specimen Dropped: "+ (System.currentTimeMillis()-startTime));

        // Back up to clear sub
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C06_CLEAR_SUB_X, C02_PLACE_SPECIMEN_Y,0,C03_TRANSITION_VELOCITY_CHILL, false, null,0,4000);
        outtake.outtakeGrab();
        // strafe over to clear sub on other side
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, C07_CLEAR_SUB_Y+C0a_FAST_STRAFE_ADJUST, C06_CLEAR_SUB_X, 0, C08_TRANSITION_VELOCITY_FAST,null, 0, 2000);

        // drive past samples
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C09_CLEAR_SAMPLE_X- C0a_FAST_STRAIGHT_ADJUST1, C07_CLEAR_SUB_Y,0,C08_TRANSITION_VELOCITY_FAST, false, null,0,4000);

        // strafe to sample 1
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, C10_SAMPLE_1_Y+C10_SAMPLE_Y_ADJUST, C09_CLEAR_SAMPLE_X, 0, C08_TRANSITION_VELOCITY_FAST,null, 0, 2000);

        // push first sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C11_DROP_SAMPLE_X+C0a_FAST_REVERSE_ADJUST, C10_SAMPLE_1_Y,0,C12_TRANSITION_VELOCITY_REVERSE, false, null,0,4000);

        // head back out to get second sample
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C09_CLEAR_SAMPLE_X- C0a_FAST_STRAIGHT_ADJUST2, C10_SAMPLE_1_Y,0,C08_TRANSITION_VELOCITY_FAST, false, null,0,4000);
        drive.strafeToTarget(270,0,C08_TRANSITION_VELOCITY_FAST, C13_SAMPLE_2_Y+C10_SAMPLE_Y_ADJUST,2000);

        // push second sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C11_DROP_SAMPLE_X+C0a_FAST_REVERSE_ADJUST, C13_SAMPLE_2_Y,0,C12_TRANSITION_VELOCITY_REVERSE, false, null,0,4000);

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C15_BACK_OUT_OBSERVATION_ZONE, C13_SAMPLE_2_Y,0,C08_TRANSITION_VELOCITY_FAST, false, null,0,4000);

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, D12_PICKUP_X,C13_SAMPLE_2_Y,0,D13_GRAB_SPECIMEN_END_VELOCITY, false, null,0,4000);
        teamUtil.pause(D14_SPECIMEN_GRAB_TIME);
        // head back out for 3rd sample
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C09_CLEAR_SAMPLE_X- C0a_FAST_STRAIGHT_ADJUST2, C13_SAMPLE_2_Y,0,C08_TRANSITION_VELOCITY_FAST,null,0,4000);
        drive.strafeToTarget(270,0,C08_TRANSITION_VELOCITY_FAST, C14_SAMPLE_3_Y+C10_SAMPLE_Y_ADJUST,2000);

        // push 3rd sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C11_DROP_SAMPLE_X+C0a_FAST_REVERSE_ADJUST, C14_SAMPLE_3_Y,0,C12_TRANSITION_VELOCITY_REVERSE,null,0,4000);
        drive.stopMotors(); // temp

    }

    public boolean autoV1Specimen(int blocks,boolean ascent){
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));

        drive.setRobotPosition(0,0,0);
        long startTime = System.currentTimeMillis();

        outtake.deployArm();

        //First move to gets robot over to side in order get to submersible fast enough
        //drive.moveTo(BasicDrive.MAX_VELOCITY,B02_PLACE_SPECIMEN_Y,B02_PLACE_SPECIMEN_Y,0,B10_END_VELOCITY_SPECIMEN,null,0,5000);
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, B01_PLACE_SPECIMEN_X, B02_PLACE_SPECIMEN_Y, 0, B03_END_VELOCITY_SPECIMEN, false, null, 0,3000);
        drive.setMotorPower(B04_SPECIMEN_MOTOR_POWER);
        teamUtil.pause(B05_SPECIMEN_PAUSE);
        drive.stopMotors();

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A04_MOVE_TO_SAMPLE_X, B02_PLACE_SPECIMEN_Y,0,500, false, null,0,4000);
        outtake.outtakeGrab();
        drive.moveTo(BasicDrive.MAX_VELOCITY,B08_GO_TO_SAMPLE_Y,B07_GO_TO_SAMPLE_X,0,B09_END_VELOCITY_SEEK_AFTER_BACKUP,null,0,5000);

        //intake.goToSeekNoWait(5000);

        if(teamUtil.alliance == RED) intake.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
        else intake.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
        //if(!intake.goToSampleAndGrab(5000)){
        //    teamUtil.log("FAILED to intake sample.  Giving up");
        //    return false;
        //}

        intake.goToUnload(5000);
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, B11_WALL_SPECIMEN_X, B12_WALL_SPECIMEN_Y,0,200, false, null,0,4000);
        output.dropSampleOutBackNoWait();

        drive.setMotorPower(-B13_SPECIMENDROP_MOTOR_POWER);
        teamUtil.pause(1000);
        drive.stopMotors();




        return true;
    }

    public boolean autoV2Specimen(int cycles){
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));
        drive.setRobotPosition(0,0,0);
        specimenCollectBlocks();
        for(int i = 1; i<=cycles;i++){
            teamUtil.log("Auto V2 Specimen Cycle Number: " + i);
            specimenCycle(i);
        }
        drive.stopMotors();
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

