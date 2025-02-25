package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.AxonSlider;
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
    public enum Ops {Specimen,
        Sample,
        Hang
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

                }
                if(driverGamepad.wasDownPressed()) {
                    long startTime = System.currentTimeMillis();
                    robot.drive.setRobotPosition(0,0,0);
                    robot.drive.setMotorsBrake();
                    robot.specimenCollectBlocksV3(false);
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
                            case 1 : robot.specimenCycleV4(1, Robot.G33_6_CYCLE_Y_PLACEMENTS[0],false, true); break;
                            case 2 : robot.specimenCycleV4(2, Robot.G33_6_CYCLE_Y_PLACEMENTS[1],false,  true); break;
                            case 3 : robot.specimenCycleV4(3, Robot.G33_6_CYCLE_Y_PLACEMENTS[2],false,  true); break;
                            case 4 : robot.specimenCycleV4(4, Robot.G33_6_CYCLE_Y_PLACEMENTS[3],false,  true); break;
                            case 5 : robot.specimenCycleV4(5, Robot.G33_6_CYCLE_Y_PLACEMENTS[4],false,  true); break;
                        }
                    }
                    robot.drive.stopMotors();
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasAPressed()){
                    long startTime = System.currentTimeMillis();
                    robot.autoV4Specimen();
                    elapsedTime = System.currentTimeMillis()-startTime;
                }
                if(driverGamepad.wasBPressed()){
                    long startTime = System.currentTimeMillis();
                    if (GRAB_SAMPLE) {
                        robot.placeFirstSpecimenV2(true);
                        robot.deliverFirstSample();
                        robot.specimenCyclePlace(1,robot.G33_6_CYCLE_Y_PLACEMENTS[0]);
                        robot.drive.stopMotors();
                    } else {
                        robot.placeFirstSpecimenV2(false);
                        robot.drive.driveMotorsHeadingsFRPower(180, 0, 1);
                        teamUtil.pause(250);
                        robot.drive.stopMotors();
                    }
                    elapsedTime = System.currentTimeMillis()-startTime;
                }

            } else if (AA_Operation == Ops.Sample) {
                ////////////////////// Testing SAMPLE Auto

                if (driverGamepad.wasYPressed()) {
                    robot.outtake.outtakeRest();
                    teamUtil.pause(1000);
                    robot.output.outputLoad(3000);
                    robot.output.bucket.setPosition(Output.BUCKET_RELOAD);
                    teamUtil.pause(1000);
                    robot.outtake.secondCalibrate();
                }
                if (driverGamepad.wasXPressed()) {
                    long startTime = System.currentTimeMillis();
                    robot.sampleAutoV3();
                    elapsedTime = System.currentTimeMillis() - startTime;
                }
                if (driverGamepad.wasRightPressed()) {
                    long startTime = System.currentTimeMillis();
                    robot.bucketToSub();
                    elapsedTime = System.currentTimeMillis() - startTime;
                    teamUtil.pause(500);
                    robot.drive.stopMotors();
                }
                if(driverGamepad.wasLeftPressed()){
                    long startTime = System.currentTimeMillis();
                    robot.subToBucket();
                    elapsedTime = System.currentTimeMillis() - startTime;
                    teamUtil.pause(500);
                    robot.drive.stopMotors();
                }

                if(driverGamepad.wasUpPressed()){
                    robot.autoGoToLoad(3000);
                }
                if(driverGamepad.wasDownPressed()){
                    robot.sampleAutoUnloadHighBucket();
                }
                if(driverGamepad.wasBPressed()){
                    robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
                    robot.intake.goToSampleAndGrabV3(false,false,true);
                }

                if(driverGamepad.wasAPressed()){
                    robot.outtake.outtakeRest();
                }

            } else {
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
                    robot.hang.clearHangServos();
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
                    robot.hang.hang_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.hang.hang_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                robot.hang.joystickDriveV2(gamepad1.left_stick_x, gamepad1.left_stick_y);
                robot.dropLiftWhenNeeded();
                robot.stowHangWhenNeeded();
                //robot.moveHookArmWhenNeeded();
                //teamUtil.log("Hangleft: " + robot.hang.hang_Left.getCurrentPosition()+ " Hangright: "+ robot.hang.hang_Right.getCurrentPosition());
                telemetry.addLine("Hangleft: " + robot.hang.hang_Left.getCurrentPosition() + " Hangright: " + robot.hang.hang_Right.getCurrentPosition());

            }

            robot.drive.odo.update();
            robot.drive.driveMotorTelemetry();
            telemetry.addLine("Running Tests " );
            telemetry.addLine("Last Auto Elapsed Time: " + elapsedTime);
            telemetry.update();
        }
    }
}
