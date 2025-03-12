package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.assemblies.Intake.EXTENDER_HOLD_RETRACT_VELOCITY;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.Arrays;

@Config

@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    Robot robot;
    TeamGamepad gamepad;




    int delay = 0;
    boolean cycle = true;
    int cycleDelay = 0;
    boolean ascent = false;
    int blocks = 1;
    int specimen = 4;


    public void initializeRobot() {
        telemetry.addLine("Initializing Robot");
        teamUtil.telemetry.update();
        telemetry.update();
        robot = new Robot();
        robot.initialize();
    }



    @Override
    public void runOpMode() {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        teamUtil.alliance = teamUtil.Alliance.RED;

        initializeRobot();
        teamUtil.robot = robot;

        while (!gamepad.wasAPressed()&&!isStopRequested()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Press Bumpers To Change Alliance");
            if (gamepad.wasRightBumperPressed() || gamepad.wasLeftBumperPressed()) {
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                }
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad.wasAPressed()&&!isStopRequested()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);

            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Press Bumpers To Change SIDE");
            if (gamepad.wasRightBumperPressed() || gamepad.wasLeftBumperPressed()) {
                if (teamUtil.SIDE == teamUtil.SIDE.BASKET) {
                    teamUtil.SIDE = teamUtil.SIDE.OBSERVATION;
                } else {
                    teamUtil.SIDE = teamUtil.SIDE.BASKET;

                }
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        //Calibrate CV

        robot.initCV(false);
        robot.intake.lightsOn();
        robot.intake.startCVPipeline();
        if(teamUtil.SIDE == teamUtil.SIDE.OBSERVATION) robot.intake.setTargetColor(teamUtil.alliance == RED ? OpenCVSampleDetectorV2.TargetColor.RED : OpenCVSampleDetectorV2.TargetColor.BLUE);
        else{
            robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        }
        if (isStopRequested()) return;

        while (!gamepad.wasAPressed()&&!isStopRequested()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Found One" + robot.intake.sampleDetector.foundOne.get());

            teamUtil.telemetry.addLine("Press X on Game Pad 1 to Move On");
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        robot.intake.lightsOff();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        robot.intake.stopCVPipeline();



        //Calibrate
        while (!gamepad.wasAPressed()&&!isStopRequested()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Check Robot and THEN");
            teamUtil.telemetry.addLine("Press X on Game Pad 1 to CALIBRATE");
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        robot.calibrate();


        int[] samplePickup = {2,2};
        int startingSampleIndex  = 0;
        if(teamUtil.SIDE == teamUtil.Side.BASKET){
            while (!gamepad.wasAPressed()&&!isStopRequested()) {
                gamepad.loop();
                teamUtil.telemetry.addLine("Check Robot and THEN");
                teamUtil.telemetry.addLine("Press X on Game Pad 1 to PREPARE OUTTAKE AND BUCKET");
                teamUtil.telemetry.update();
            }
            if (isStopRequested()) return;

            robot.outtake.outtakeRest();
            teamUtil.pause(1000);
            robot.output.outputLoad(3000);
            robot.output.bucket.setPosition(Output.BUCKET_TRAVEL);
            teamUtil.pause(1000);
            robot.outtake.secondCalibrate();
            while (!gamepad.wasAPressed()&&!isStopRequested()) {
                gamepad.loop();

                if(gamepad.wasUpPressed()){
                    samplePickup[startingSampleIndex]+=1;
                    if(samplePickup[startingSampleIndex]>3){
                        samplePickup[startingSampleIndex]=1;
                    }
                }
                if(gamepad.wasDownPressed()){
                    samplePickup[startingSampleIndex]-=1;
                    if(samplePickup[startingSampleIndex]<1){
                        samplePickup[startingSampleIndex]=3;
                    }
                }
                if(gamepad.wasLeftPressed()){
                    startingSampleIndex-=1;
                    if(startingSampleIndex<0){
                        startingSampleIndex=1;
                    }
                }
                if(gamepad.wasRightPressed()){
                    startingSampleIndex+=1;
                    if(startingSampleIndex>1){
                        startingSampleIndex=0;
                    }
                }

                teamUtil.telemetry.addLine("Press X on Game Pad 1 to CONFIRM NUMBERS AND MOVE ON");

                teamUtil.telemetry.addLine("Press RIGHT/LEFT on DPAD to CHANGE BLOCK INDEX");
                teamUtil.telemetry.addLine("Press UP/DOWN on DPAD to TO CHANGE WHICH BLOCK");
                if(startingSampleIndex==0) teamUtil.telemetry.addLine("CHANGING FIRST BLOCK");
                else teamUtil.telemetry.addLine("CHANGING SECOND BLOCK");
                teamUtil.telemetry.addLine("//////////////////////////////////////////////");


                teamUtil.telemetry.addLine("CURRENT SAMPLE POSITION ARRAY: " + Arrays.toString(samplePickup));


                teamUtil.telemetry.update();
            }
        }
        if (isStopRequested()) return;



        int teethXOffset = 3;
        int teethYOffset = 2;

        if(teamUtil.SIDE == teamUtil.Side.OBSERVATION){
            //first extender tooth 155
            //second is 216, third isi 284, fourth is 358, fifth is 426,
            while (!gamepad.wasAPressed()&&!isStopRequested()) {
                gamepad.loop();
                teamUtil.telemetry.addLine("Press Up To Increase X By A Tooth");
                teamUtil.telemetry.addLine("Press Down To Decrease X By A Tooth");

                teamUtil.telemetry.addLine("Press Right To Increase Y By A Tooth");
                teamUtil.telemetry.addLine("Press Down To Decrease Y By A Tooth");

                if(gamepad.wasUpPressed()){
                    teethXOffset+=1;
                }
                if(gamepad.wasDownPressed()){
                    teethXOffset-=1;
                }
                if(gamepad.wasLeftPressed()){
                    teethYOffset-=1;
                }
                if(gamepad.wasRightPressed()){
                    teethYOffset+=1;
                }

                teamUtil.telemetry.addLine("Tooth X Offset (EXTENDER DIRECTION!!!): " + teethXOffset);
                teamUtil.telemetry.addLine("Tooth Y Offset (SLIDER DIRECTION!!!): " + teethYOffset);
                teamUtil.telemetry.addLine("Calculated Extender Position: " + robot.extenderTeethToEncoder(teethXOffset));
                teamUtil.telemetry.addLine("Calculated Slider Position: " + robot.extenderTeethToEncoder(teethYOffset));

                teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
                teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);

                teamUtil.telemetry.addLine("Press X to Move On");

                teamUtil.telemetry.update();
            }
        }
        robot.nextExtenderPos = robot.extenderTeethToEncoder(teethXOffset);
        robot.nextSliderPos = robot.sliderTeethToEncoder(teethYOffset);
        if (isStopRequested()) return;


        //if (teamUtil.SIDE == teamUtil.SIDE.BASKET) { // get camera running but only if we will use it
        //robot.initCV(false); // no live stream enabled means better FPS
        //}
        if(teamUtil.alliance == RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.SINELON_RED);
        }else{
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.SINELON_BLUE);
        }
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addLine("Ready to Go!");
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            if(teamUtil.SIDE == teamUtil.Side.OBSERVATION){
                teamUtil.telemetry.addLine("Tooth X Offset: " + teethXOffset);
                teamUtil.telemetry.addLine("Tooth Y Offset: " + teethYOffset);
                teamUtil.telemetry.addLine("Calculated Extender Position: " + robot.extenderTeethToEncoder(teethXOffset));
                teamUtil.telemetry.addLine("Calculated Slider Position: " + robot.sliderTeethToEncoder(teethYOffset));
            }else{
                teamUtil.telemetry.addLine("Block Positions: " + Arrays.toString(samplePickup));
            }


            if (teamUtil.SIDE == teamUtil.Side.BASKET) {
                telemetry.addLine("Running Basket");

            } else {
                telemetry.addLine("Running Specimen");
            }
            telemetry.update();
        }
        if (isStopRequested()) return;

        waitForStart();
        teamUtil.inInitialization=false;
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        if(!isStopRequested()) {
            long startTime = System.currentTimeMillis();
            teamUtil.pause(delay);
            robot.intake.extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);

            if (teamUtil.SIDE == teamUtil.Side.BASKET) {
                robot.sampleAutoV4(samplePickup);
            } else {
                robot.autoV5Specimen();
            }
            robot.drive.stopMotors();
            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);

            while (opModeIsActive()) { // don't kill opMode until the last possible moment to allow other threads to finish
            }

            teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
            //TODO stop sample opMode right after its done dont wait
        }
    }
}


