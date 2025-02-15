package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.assemblies.Intake.EXTENDER_HOLD_RETRACT_VELOCITY;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "JudgingOpMode", group = "LinearOpMode")
public class JudgingOpMode extends LinearOpMode {

    Robot robot;
    Blinkin blinkin;
    TeamGamepad driverGamepad;
    boolean endgame = false;
    double EXTENDER_Y_DEADBAND = 0.3;
    double SLIDER_X_DEADBAND = 0.3;
    boolean lowBucketToggle= false;
    int optionsPresses = 0;
    boolean hangManualControl= false;
    public static boolean initCV= false;
    boolean outtakeUp = false;



    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;





    


    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);



        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        robot = new Robot();
        robot.initialize();
        robot.initCV(initCV);// TODO: false for competition

        if (!teamUtil.justRanAuto&&!teamUtil.justRanCalibrateRobot) { // Auto already took care of this, so save time and don't move anything!
            robot.calibrate();

        }
        teamUtil.justRanAuto=false;
        teamUtil.justRanCalibrateRobot=false;





        
        
        
        waitForStart();

        // Lock extender in its current position
        robot.intake.extender.setTargetPosition(robot.intake.extender.getCurrentPosition());
        robot.intake.extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.intake.extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);

        robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        while (opModeIsActive()){

            teamUtil.theBlinkin.setSignal(Blinkin.Signals.JUDGING_BLINKIN);
            driverGamepad.loop();
            
            //ARMS GAMEPAD
            //Outake
            if(driverGamepad.wasUpPressed()){
                robot.outtake.deployArm();
            }

            if(driverGamepad.wasDownPressed()){
                robot.dropSampleOutBackAndArmGrabNoWait(3000);
            }
            if(driverGamepad.wasLeftPressed()){
                robot.outtake.outtakeRest();
            }
            if(driverGamepad.wasRightPressed()){
                robot.outtake.outakearm.setPosition(robot.outtake.ARM_DOWN);
                robot.outtake.outakewrist.setPosition(robot.outtake.WRIST_GRAB);
            }

            if (driverGamepad.wasAPressed() && !robot.intake.autoSeeking.get()) {
                if(robot.intake.extender.getCurrentPosition()<robot.intake.EXTENDER_GO_TO_SEEK_THRESHOLD){
                    //robot.intake.unloadV2NoWait(true); (fast unload)
                    robot.intake.safeUnloadNoWait();
                }
                else{
                    robot.intake.extenderSafeRetractNoWait(4000);
                }
            }

            if(!robot.intake.autoSeeking.get()) {
                robot.intake.manualX(driverGamepad.gamepad.left_stick_x);
            }
            if ((Math.abs(driverGamepad.gamepad.left_stick_y) > EXTENDER_Y_DEADBAND) && !robot.intake.autoSeeking.get()) {
                robot.intake.manualY(driverGamepad.gamepad.left_stick_y);
            }
            if (Math.abs(driverGamepad.gamepad.right_stick_y) > EXTENDER_Y_DEADBAND) {
                robot.intake.manualYNoSeek(driverGamepad.gamepad.right_stick_y);
            }

            if (driverGamepad.gamepad.right_stick_button) {
                robot.intake.grabberReady();
            }


            if ((driverGamepad.wasBPressed()&&teamUtil.alliance == teamUtil.Alliance.RED)&&!robot.intake.autoSeeking.get()) { //Grab Red
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToSampleAndGrabNoWaitV3(false); // TODO was true for auto unload--switched to false to try intake-only collection
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false);
                }
            }
            if ((driverGamepad.wasYPressed())&&!robot.intake.autoSeeking.get()) { //Grab Yellow
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.goToSampleAndGrabAndLiftToBucketNoWait(true);
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false);
                }
            }
            if ((driverGamepad.wasXPressed()&&teamUtil.alliance == teamUtil.Alliance.BLUE)&&!robot.intake.autoSeeking.get()) { //Grab Blue
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToSampleAndGrabNoWaitV3(false); // TODO was true for auto unload--switched to false to try intake-only collection
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false);
                }
            }


            //OUTPUT
            if (driverGamepad.wasRightTriggerPressed()) {
                robot.dropSampleOutBackWithFlipperResetNoWait();
            }
            if(driverGamepad.wasHomePressed()){
                if(lowBucketToggle){
                    lowBucketToggle=false;
                }else{
                    lowBucketToggle=true;

                }
            }
            if (driverGamepad.wasLeftTriggerPressed()) {
                if(!lowBucketToggle){
                    robot.output.outputHighBucketNoWait(3000);
                }else{
                    robot.output.outputLowBucketNoWait(3000);

                }
            }
            if (driverGamepad.wasLeftBumperPressed()) {
                robot.output.outputLoadNoWait(4000);
            }

            //HANG
            if(driverGamepad.wasOptionsPressed()) {
                robot.hangPhase1NoWait();
            }







            robot.outputTelemetry();
            robot.drive.odo.update();
            //telemetry.addData("Left Hang Velocity", robot.hang.hang_Left.getVelocity());
            //telemetry.addData("Right Hang Velocity", robot.hang.hang_Right.getVelocity());
            //telemetry.addLine("Low Bucket Toggled: " + lowBucketToggle);
            //telemetry.addLine("Hang Manual: " + hangManualControl);
            telemetry.update();


        }
    }
}
