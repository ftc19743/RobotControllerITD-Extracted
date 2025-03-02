package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.assemblies.Intake.EXTENDER_HOLD_RETRACT_VELOCITY;
import static org.firstinspires.ftc.teamcode.assemblies.Intake.FLIPPER_UNLOAD;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class FiveSpecimenAutoV4 {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public BasicDrive drive;
    public Output output;
    public Outtake outtake;
    public Intake intake;
    public Hang hang;
    public Blinkin blinkin;

    public static boolean details = false;

    public FiveSpecimenAutoV4() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }
    // set up pointers to the robot sub classes so everything can work here
    public void init (Robot robot) {
        drive = robot.drive;
        output = robot.output;
        intake = robot.intake;
        outtake = robot.outtake;
        blinkin = robot.blinkin;
        hang = robot.hang;
    }


    public boolean autoV4Specimen(){
        teamUtil.log("Running 5 Specimen Auto V4.  Alliance: " + (teamUtil.alliance == teamUtil.Alliance.RED ? "RED" : "BLUE"));

        // make sure extender isn't going anywhere
        intake.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.extender.setTargetPosition(Intake.EXTENDER_CALIBRATE);
        intake.extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);
        intake.flipper.setPosition(FLIPPER_UNLOAD);

        drive.setRobotPosition(0,0,0);
        specimenCollectBlocksV2();
        fiveSpecimenCycle();
        drive.stopMotors();
        //TODO make it so AUTO_INTAKE_SPECIMEN is an option when initializing
        return true;
    }

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
/*
        //drive.stopMotors(); // faster to just reverse motors at full speed
        //teamUtil.pause(F12_REVERSE_BRAKING_PAUSE);
 */
        // TODO: CHANGE
        drive.stopMotors(); // faster to just reverse motors at full speed
        teamUtil.pause(F12_REVERSE_BRAKING_PAUSE);

        // head back out to get second sample
        // TODO: CHANGE
        //drive.lastVelocity = BasicDrive.MAX_VELOCITY; // make motors start at full power // was in effect
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F09_CLEAR_SAMPLE_X- F0a_FAST_STRAIGHT_ADJUST2, F10_SAMPLE_1_Y,0,F08_TRANSITION_VELOCITY_FAST, false, null,0,4000);
        drive.strafeToTarget(270,0,F08_TRANSITION_VELOCITY_FAST, F13_SAMPLE_2_Y+F10_SAMPLE_Y_ADJUST,2000);

        // push second sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, F13a_DROP_SAMPLE2_X +F0a_FAST_REVERSE_ADJUST, F13_SAMPLE_2_Y,0,BasicDrive.MAX_VELOCITY, false, null,0,4000);

        // TODO: CHANGE
        drive.stopMotors(); // faster to just reverse motors at full speed // was commented out
        teamUtil.pause(F12_REVERSE_BRAKING_PAUSE);                         // was commented out

        // head back out for 3rd sample
        //drive.lastVelocity = BasicDrive.MAX_VELOCITY; // make motors start at full power // was in effect
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

    static public int[] F33_5_CYCLE_Y_PLACEMENTS = {50, 68, 112, 145};
    public void fiveSpecimenCycle() {
        specimenCycleV3(1, F33_5_CYCLE_Y_PLACEMENTS[0],false, true, true);
        specimenCycleV3(2, F33_5_CYCLE_Y_PLACEMENTS[1],false, true, true);
        specimenCycleV3(3, F33_5_CYCLE_Y_PLACEMENTS[2],false, true, true);
        specimenCycleV3(4, F33_5_CYCLE_Y_PLACEMENTS[3],false, true, true);
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

                // TODO: CHANGE
                //drive.driveMotorsHeadingsFR(180,0,BasicDrive.MAX_VELOCITY);
                drive.stopMotors();

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
            /*
            if(teamUtil.alliance == RED){
                intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
            }
            else{
                intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
            }
            grabbedSample=intake.goToSampleAndGrabV3(false, false);
            autoRetractAndUnloadNoWait(grabbedSample);
            intake.lightsOff();

             */
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
                    //if (grabSample) dropSampleOutBackWithFlipperResetNoWait();
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

}
