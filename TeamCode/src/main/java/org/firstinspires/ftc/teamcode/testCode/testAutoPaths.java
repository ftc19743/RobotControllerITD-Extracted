package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "testAutoPaths", group = "LinearOpMode")
public class testAutoPaths extends LinearOpMode {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot robot;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean useArms = false;
    public static boolean liveStream = true;
    public static int BLOCKS = 2;
    public static int END_CYCLES = 0;
    public static int START_CYCLE = 1;
    public static boolean GRAB_SAMPLE = false;
    public static boolean useCV = true;
    public static boolean deliverFirstSampleV2 = false;

    public static int teethXOffset = 0;
    public static int teethYOffset = 0;
    public static int[] position = {2 ,2};

    public enum Ops {Specimen,
        Sample,
        Hang,
        TestMovements
    };
    public static testAutoPaths.Ops AA_Operation = Ops.Specimen;
    public long elapsedTime;
    private boolean enableLiveView = false;



    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        driverGamepad.reset();
        armsGamepad.reset();
        telemetry.addLine("Initializing.  Please wait.");
        telemetry.update();
        robot = new Robot();
        teamUtil.robot = robot;

        robot.initialize();
        if (useCV) {
            telemetry.addLine("Initializing CV.  Please wait.");
            telemetry.update();
            robot.initCV(liveStream);
        }
        telemetry.addLine("Calibrating.  Please wait.");
        telemetry.update();
        robot.calibrate();

        telemetry.addLine("Ready to start");
        telemetry.update();
        robot.drive.setHeading(180);

        driverGamepad.reset();
        armsGamepad.reset();
        waitForStart();
        teamUtil.inInitialization = false;

        driverGamepad.reset();
        armsGamepad.reset();
        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();
            telemetry.addLine("Alliance: "+ teamUtil.alliance);
            telemetry.addLine("Testing: " + AA_Operation);

            telemetry.addLine("Use Arms: "+ useArms);

            telemetry.addLine("Strafe: "+ robot.drive.odo.getPosY());
            telemetry.addLine("Forward: "+ robot.drive.odo.getPosX());



            if (driverGamepad.gamepad.left_stick_button) {
                teamUtil.logSystemHealth();
            }

            if (driverGamepad.wasLeftBumperPressed()) {
                if (teamUtil.alliance== RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = RED;
                }
            }
            if (driverGamepad.wasRightBumperPressed()) {
                if (AA_Operation == Ops.Specimen) {
                    AA_Operation = Ops.Sample;
                } else if (AA_Operation == Ops.Sample) {
                    AA_Operation = Ops.Hang;
                } else if (AA_Operation == Ops.Hang) {
                    AA_Operation = Ops.Specimen;
                }
            }

            if(driverGamepad.wasRightTriggerPressed()) {
                robot.drive.setRobotPosition(0,0,0);
            }
            if(driverGamepad.wasLeftTriggerPressed()){
                robot.resetRobot();
            }

            ////////////////// Testing Specimen Auto
            if (AA_Operation == Ops.Specimen) {

                if(driverGamepad.wasUpPressed()) {
                    long startTime = System.currentTimeMillis();
                    //robot.autoV4Specimen();
                    robot.specimenCycleV4(5, Robot.G33_6_CYCLE_Y_PLACEMENTS[4],false,  0, false);
                    robot.park();
                    robot.drive.stopMotors();
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasDownPressed()) {
                    long startTime = System.currentTimeMillis();
                    robot.drive.setRobotPosition(0,0,0);
                    robot.drive.setMotorsBrake();
                    robot.specimenCollectBlocksV3();
                    robot.drive.stopMotors();
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasYPressed()){
                    robot.outtake.outtakeGrab();
                }
                if(driverGamepad.wasXPressed()) {
                    robot.nextExtenderPos = Intake.EXTENDER_AUTO_START_SEEK;
                    long startTime = System.currentTimeMillis();
                    for(int i = START_CYCLE; i<=END_CYCLES;i++){
                        teamUtil.log("Auto V4 Specimen Cycle Number: " + i);
                        switch (i) {
                            case 1 : robot.specimenCycleV4(1, Robot.G33_6_CYCLE_Y_PLACEMENTS[0],false, 0, true); break;
                            case 2 : robot.specimenCycleV4(2, Robot.G33_6_CYCLE_Y_PLACEMENTS[1],true,  Robot.G33_6_CYCLE_SHIFT_2, true); break;
                            case 3 : robot.specimenCycleV4(3, Robot.G33_6_CYCLE_Y_PLACEMENTS[2],false,  0, true); break;
                            case 4 : robot.specimenCycleV4(4, Robot.G33_6_CYCLE_Y_PLACEMENTS[3],false,  0, true); break;
                            case 5 : robot.specimenCycleV4(5, Robot.G33_6_CYCLE_Y_PLACEMENTS[4],false,  0, false); break;
                        }
                    }
                    robot.drive.stopMotors();
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasAPressed()){
                    long startTime = System.currentTimeMillis();
                    robot.nextExtenderPos = robot.extenderTeethToEncoder(teethXOffset);
                    robot.nextSliderPos = robot.sliderTeethToEncoder(teethYOffset);
                    robot.autoV5Specimen();
                    elapsedTime = System.currentTimeMillis()-startTime;
                    robot.drive.stopMotors();
                }
                if(driverGamepad.wasHomePressed()){
                    robot.nextExtenderPos = robot.extenderTeethToEncoder(teethXOffset);
                    robot.nextSliderPos = robot.sliderTeethToEncoder(teethYOffset);
                    long startTime = System.currentTimeMillis();
                    robot.drive.setRobotPosition(0,0,0);
                    robot.placeFirstSpecimenV2(true);
                    if(!deliverFirstSampleV2){
                        robot.deliverFirstSample();
                    }
                    else{
                        robot.deliverFirstSampleV2();
                    }
                    robot.drive.stopMotors();
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasBPressed()){
                    long startTime = System.currentTimeMillis();
                    if (GRAB_SAMPLE) {
                        robot.drive.setRobotPosition(0,0,0);
                        robot.placeFirstSpecimenV2(true);
                        robot.deliverFirstSample();
                        //robot.specimenCyclePlace(1,robot.G33_6_CYCLE_Y_PLACEMENTS[0]);
                        robot.drive.stopMotors();
                    } else {
                        robot.placeFirstSpecimenV2(false);
                        robot.drive.driveMotorsHeadingsFRPower(180, 0, 1);
                        teamUtil.pause(250);
                        robot.drive.stopMotors();
                    }
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasLeftPressed()){
                    robot.outtake.outtakeRest();
                    teamUtil.pause(2000);
                    robot.output.bucket.setPosition(Output.BUCKET_IDLE);
                    teamUtil.pause(1000);
                    robot.outtake.secondCalibrate();
                }

            } else if (AA_Operation == Ops.Sample) {
                ////////////////////// Testing SAMPLE Auto

                if (driverGamepad.wasYPressed()) {
                    robot.outtake.outtakeRest();
                    teamUtil.pause(1000);
                    robot.output.outputLoad(3000);
                    robot.output.bucket.setPosition(Output.BUCKET_TRAVEL);
                    teamUtil.pause(1000);
                    robot.outtake.secondCalibrate();
                }
                if (driverGamepad.wasXPressed()) {
                    long startTime = System.currentTimeMillis();
                    robot.sampleAutoV4(position);
                    elapsedTime = System.currentTimeMillis() - startTime;
                    robot.drive.stopMotors();
                }
                if (driverGamepad.wasRightPressed()) {
                    long startTime = System.currentTimeMillis();
                    robot.bucketToSubV3(GRAB_SAMPLE ? 2: 0);
                    if (GRAB_SAMPLE) {
                        robot.intake.autoGoToSampleAndGrabV3(false,false,true,1500);
                    }

                    elapsedTime = System.currentTimeMillis() - startTime;
                    teamUtil.pause(500);
                    robot.drive.stopMotors();
                }
                if(driverGamepad.wasLeftPressed()){
                    long startTime = System.currentTimeMillis();
                    robot.subToBasketV2(GRAB_SAMPLE);
                    elapsedTime = System.currentTimeMillis() - startTime;
                    teamUtil.pause(500);
                    robot.drive.stopMotors();
                }
                if(driverGamepad.wasHomePressed()){
                    long startTime = System.currentTimeMillis();
                    robot.bucketToSubV3(GRAB_SAMPLE ? 2: 0);
                    if (GRAB_SAMPLE) {
                        robot.intake.autoGoToSampleAndGrabV3(false,false,true,3000);
                    }
                    robot.subToBasketV2(GRAB_SAMPLE);
                    elapsedTime = System.currentTimeMillis() - startTime;
                    teamUtil.pause(500);
                    robot.drive.stopMotors();
                }

                if(driverGamepad.wasUpPressed()){
                    long startTime = System.currentTimeMillis();
                    // robot.bucketCycle();
                    robot.autoAscentPark();
                    elapsedTime = System.currentTimeMillis() - startTime;
                    teamUtil.pause(500);
                    robot.drive.stopMotors();
                }
                if(driverGamepad.wasDownPressed()){
                    //robot.sampleAutoUnloadHighBucket(false);
                    long startTime = System.currentTimeMillis();
                    robot.drive.setRobotPosition(0,0,0);
                    robot.deliver4Samples();
                    robot.drive.stopMotors();
                    elapsedTime = System.currentTimeMillis() - startTime;
                }
                if(driverGamepad.wasBPressed()){
                    robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
                    robot.intake.goToSampleAndGrabV3(true,true,true);
                }

                if(driverGamepad.wasAPressed()){
                    robot.outtake.outtakeRest();
                }

            } else if (AA_Operation == Ops.Hang) {
                ////////////////////////////////////////////////////////////////////////////
                // TESTING HANG
                if (driverGamepad.wasAPressed()) {
                    robot.hangPhase1();
                }
                if (driverGamepad.wasBPressed()) {
                    robot.hang.calibrate();
                }
                if (driverGamepad.wasYPressed()) {
                    robot.hangPhase2V3();
                }
                if(driverGamepad.wasRightPressed()){
                    robot.hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if( driverGamepad.wasUpPressed()){
                    robot.hang.extendHang();
                }if( driverGamepad.wasDownPressed()){
                    robot.hang.engageHangV2();
                }if( driverGamepad.wasLeftPressed()){
                    robot.hang.stowHang();
                }
                if(driverGamepad.wasOptionsPressed()){
                    robot.pickUpHooks();
                }
                if(driverGamepad.wasHomePressed()){
                    robot.hang.stowHookGrabber();
                }
                if (driverGamepad.wasXPressed()) {
                    robot.hangPhase2Level2();
                }
                robot.hang.joystickDriveV2(gamepad1.left_stick_x, gamepad1.left_stick_y);
                robot.hangPhase2DelayedOps();

                //teamUtil.log("Hangleft: " + robot.hang.hang_Left.getCurrentPosition()+ " Hangright: "+ robot.hang.hang_Right.getCurrentPosition());
                telemetry.addLine("Hangleft: " + robot.hang.hang_Left.getCurrentPosition() + " Hangright: " + robot.hang.hang_Right.getCurrentPosition());

            } else { // Testing various movements
                if (driverGamepad.wasYPressed()) {
                    robot.intake.flipperGoToSeek(2000);
                    robot.intake.extendersToPositionMaxVelo(Robot.A02_SAMPLE_2_EXTENDER, 2000);
                }
                if (driverGamepad.wasAPressed()) {
                    robot.intake.goToUnload(2000);
                }
                if (driverGamepad.wasUpPressed()) { // sample 1 pickup
                    long startTime = System.currentTimeMillis();

                    robot.drive.moveToPower(Robot.A06_1_SAMPLE_PICKUP_POWER, Robot.A06_1_SAMPLE_PICKUP_STRAFE, Robot.A06_1_SAMPLE_PICKUP_STRAIGHT, Robot.A06_1_SAMPLE_PICKUP_RH, Robot.A06_1_SAMPLE_PICKUP_POWER, null, 0, false, 3000);
                    robot.drive.stopMotors();
                    robot.drive.waitForRobotToStop(1000);
                    teamUtil.pause(Robot.A06_1_BRAKE_PAUSE);
                    elapsedTime = System.currentTimeMillis() - startTime;

                }
                if (driverGamepad.wasDownPressed()) { // sample 3 pickup
                    long startTime = System.currentTimeMillis();

                    robot.drive.moveTo(Robot.A00_MAX_SPEED_NEAR_BUCKET, Robot.A06_1_SAMPLE_PICKUP_STRAFE, Robot.A06_1_SAMPLE_PICKUP_STRAIGHT, Robot.A06_1_SAMPLE_PICKUP_RH, Robot.A00_END_SPEED, null, 0, false, 3000);
                    robot.drive.stopMotors();
                    robot.drive.waitForRobotToStop(1000);
                    teamUtil.pause(Robot.A06_1_BRAKE_PAUSE);
                    elapsedTime = System.currentTimeMillis() - startTime;
                }
                if (driverGamepad.wasRightPressed()) { // sample 2 pickup
                    long startTime = System.currentTimeMillis();

                    robot.drive.moveToPower(Robot.A08_2_SAMPLE_PICKUP_POWER, Robot.A08_2_SAMPLE_PICKUP_STRAFE, Robot.A08_2_SAMPLE_PICKUP_STRAIGHT, Robot.A08_2_SAMPLE_PICKUP_HEADING, Robot.A08_2_SAMPLE_PICKUP_POWER, null, 0, false, 3000);
                    robot.drive.stopMotors();
                    robot.drive.waitForRobotToStop(1000);
                    teamUtil.pause(Robot.A06_1_BRAKE_PAUSE);
                    elapsedTime = System.currentTimeMillis() - startTime;

                }

            }

            robot.drive.odo.update();
            robot.drive.driveMotorTelemetry();
            robot.intake.intakeTelemetry();
            telemetry.addLine("Last Auto Elapsed Time: " + elapsedTime);
            telemetry.update();
        }
    }
}
