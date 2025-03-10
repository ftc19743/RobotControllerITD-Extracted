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

        //Calibrate CV

        robot.initCV(false);
        robot.intake.lightsOn();
        robot.intake.startCVPipeline();
        if(teamUtil.SIDE == teamUtil.SIDE.OBSERVATION) robot.intake.setTargetColor(teamUtil.alliance == RED ? OpenCVSampleDetectorV2.TargetColor.RED : OpenCVSampleDetectorV2.TargetColor.BLUE);
        else{
            robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        }
        while (!gamepad.wasAPressed()&&!isStopRequested()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Found One" + robot.intake.sampleDetector.foundOne.get());

            teamUtil.telemetry.addLine("Press X on Game Pad 1 to Move On");
            teamUtil.telemetry.update();
        }
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
        robot.calibrate();

        if(teamUtil.SIDE == teamUtil.Side.BASKET){
            while (!gamepad.wasAPressed()&&!isStopRequested()) {
                gamepad.loop();
                teamUtil.telemetry.addLine("Check Robot and THEN");
                teamUtil.telemetry.addLine("Press X on Game Pad 1 to PREPARE OUTTAKE AND BUCKET");
                teamUtil.telemetry.update();
            }
            robot.outtake.outtakeRest();
            teamUtil.pause(1000);
            robot.output.outputLoad(3000);
            robot.output.bucket.setPosition(Output.BUCKET_RELOAD);
            teamUtil.pause(1000);
            robot.outtake.secondCalibrate();
        }


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
            teamUtil.telemetry.addLine("Tooth X Offset: " + teethXOffset);
            teamUtil.telemetry.addLine("Tooth Y Offset: " + teethYOffset);
            teamUtil.telemetry.addLine("Calculated Extender Position: " + robot.extenderTeethToEncoder(teethXOffset));
            teamUtil.telemetry.addLine("Calculated Slider Position: " + robot.extenderTeethToEncoder(teethYOffset));

            if (teamUtil.SIDE == teamUtil.Side.BASKET) {
                telemetry.addLine("Running Basket");

            } else {
                telemetry.addLine("Running Specimen");
            }
            telemetry.update();
        }


        waitForStart();
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        if(!isStopRequested()) {
            long startTime = System.currentTimeMillis();
            teamUtil.pause(delay);
            robot.intake.extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);

            if (teamUtil.SIDE == teamUtil.Side.BASKET) {
                //robot.sampleAutoV3(); // TODO: Change to V4 and add positions array
            } else {
                robot.autoV5Specimen();
            }
            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);

            while (opModeIsActive()) {
            }

            teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
            //TODO stop sample opMode right after its done dont wait
        }
    }
}


