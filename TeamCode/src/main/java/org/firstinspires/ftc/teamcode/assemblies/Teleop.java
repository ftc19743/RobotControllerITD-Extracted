package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.assemblies.Intake.EXTENDER_HOLD_RETRACT_VELOCITY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;
    Blinkin blinkin;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean endgame = false;
    double EXTENDER_Y_DEADBAND = 0.3;
    double SLIDER_X_DEADBAND = 0.3;
    boolean lowBucketToggle= false;
    int optionsPresses = 0;
    boolean hangManualControl= false;
    public static boolean enableLiveView = false;
    boolean outtakeUp = true;
    public static boolean proportionalBucketControl = true;


    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;






    /*
    public void loopRunTimeCalculate(int loopNumber,boolean button){
        long startTime=0;
        long endTime=0;
        int loopAmount=0;
        int buttonPressNumber=0;
        if(button&&buttonPressNumber==0){
            buttonPressNumber=1;
            startTime=System.currentTimeMillis();
        }
        if(button&&buttonPressNumber==1){
            loopAmount=loopNumber;
            endTime=System.currentTimeMillis();
        }
        long totalRunTime = endTime-startTime;
        long loopTime = totalRunTime/loopAmount;

        //TODO: take away (only for testing)
        telemetry.addLine("Button Press Number" + buttonPressNumber);

        teamUtil.log("Loop Time" + loopTime);


    }

     */
    //tentative test code


    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);



        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        robot = new Robot();
        robot.initialize();
        robot.initCV(enableLiveView);// TODO: false for competition

        if (!teamUtil.justRanAuto&&!teamUtil.justRanCalibrateRobot) { // Auto already took care of this, so save time and don't move anything!
            robot.calibrate();
            robot.drive.setHeading(0);
        }
        teamUtil.justRanAuto=false;
        teamUtil.justRanCalibrateRobot=false;



        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
        telemetry.update();



        optionsPresses=0;
        hangManualControl = false;
        boolean inHang = false;


        while (!opModeIsActive()) {
            driverGamepad.loop();
            if(driverGamepad.wasRightBumperPressed()||driverGamepad.wasLeftBumperPressed()){
                if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }else{
                    teamUtil.alliance= teamUtil.Alliance.BLUE;
                }
            }
            telemetry.addLine("Ready to start");
            telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
            telemetry.update();
        }
        //TODO: FIX ALL STATE MANAGEMENT
        waitForStart();

        // Lock extender in its current position
        robot.intake.extender.setTargetPosition(robot.intake.extender.getCurrentPosition());
        robot.intake.extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.intake.extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);
        
        robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();



            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(0);
            }

            if(driverGamepad.wasYPressed()){
                robot.drive.setHeldHeading(0);
            }
            if(driverGamepad.wasOptionsPressed()){
                robot.drive.setHeldHeading(315);
            }if(driverGamepad.wasAPressed()) {
                if (OpenCVSampleDetectorV2.targetColor == OpenCVSampleDetectorV2.TargetColor.YELLOW) {
                    robot.drive.setHeldHeading(315);
                } else {
                    robot.drive.setHeldHeading(180);
                }
            }
            if(driverGamepad.wasXPressed()){
                robot.drive.setHeldHeading(90);
            }
            if(driverGamepad.wasBPressed()){
                robot.drive.setHeldHeading(270);
            }

            //ARMS GAMEPAD
            //Outake
            if(driverGamepad.wasLeftBumperPressed()){
                if(outtakeUp){
                    robot.outtake.outtakeGrab();
                    outtakeUp = false;
                }else{
                    robot.outtake.deployArmNoWait(2000);
                    outtakeUp = true;
                }
            }

            if(armsGamepad.wasLeftPressed()){
                robot.outtake.outtakeRest();
            }

            /*
            if(armsGamepad.wasDownPressed()){
                robot.dropSampleOutBackAndArmGrabNoWait(3000);
            }

            if(armsGamepad.wasRightPressed()){
                robot.outtake.outakearm.setPosition(Outtake.ARM_DOWN);
                robot.outtake.outakewrist.setPosition(Outtake.WRIST_GRAB);
            }

             */



            if (armsGamepad.wasAPressed() && !robot.intake.autoSeeking.get()) {
                if(robot.intake.extender.getCurrentPosition()<Intake.EXTENDER_GO_TO_SEEK_THRESHOLD){
                    //robot.intake.unloadV2NoWait(true); (fast unload)
                    robot.intake.unloadToChuteNoWait();
                }
                else{
                    robot.intake.extenderSafeRetractNoWait(4000);
                }
            }

            //manual moving override
            if(armsGamepad.wasHomePressed()){
                robot.intake.moving.set(false);
            }

            if(!robot.intake.autoSeeking.get()) {
                robot.intake.manualX(armsGamepad.gamepad.left_stick_x);
            }
            if ((Math.abs(armsGamepad.gamepad.left_stick_y) > EXTENDER_Y_DEADBAND) && !robot.intake.autoSeeking.get()) {
                robot.intake.manualY(armsGamepad.gamepad.left_stick_y);
            }
            if (Math.abs(armsGamepad.gamepad.right_stick_y) > EXTENDER_Y_DEADBAND) {
                robot.intake.manualYNoSeek(armsGamepad.gamepad.right_stick_y);
            }

            if (armsGamepad.gamepad.right_stick_button) {
                robot.intake.grabberReady();
            }


            if ((armsGamepad.wasBPressed()&&teamUtil.alliance == teamUtil.Alliance.RED)&&!robot.intake.autoSeeking.get()) { //Grab Red
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToSampleAndGrabNoWaitV3(false,false); // TODO was true for auto unload--switched to false to try intake-only collection
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false,false);
                }
            }
            if ((armsGamepad.wasYPressed())&&!robot.intake.autoSeeking.get()) { //Grab Yellow
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
                robot.outtake.outtakeRest();
                robot.output.bucket.setPosition(Output.BUCKET_RELOAD);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.goToSampleAndGrabAndLiftToBucketNoWait(true);
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false,false);
                }
            }
            if ((armsGamepad.wasXPressed()&&teamUtil.alliance == teamUtil.Alliance.BLUE)&&!robot.intake.autoSeeking.get()) { //Grab Blue
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToSampleAndGrabNoWaitV3(false,false); // TODO was true for auto unload--switched to false to try intake-only collection
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false,false);
                }
            }


            //OUTPUT
            if (proportionalBucketControl && !robot.output.outputLiftAtBottom.get()) {
                robot.output.bucket.setPosition(Output.BUCKET_TRAVEL - armsGamepad.gamepad.right_trigger * (Output.BUCKET_TRAVEL - Output.BUCKET_DEPLOY_AT_TOP)) ;
                armsGamepad.resetWasRightTriggerPressed();
            } else {
                if (armsGamepad.wasRightTriggerPressed()) {
                    robot.output.dropSampleOutBackNoWait();
                }
            }
            if(armsGamepad.wasRightBumperPressed()){
                if(lowBucketToggle){
                    lowBucketToggle=false;
                }else{
                    lowBucketToggle=true;

                }
            }
            if (armsGamepad.wasLeftTriggerPressed()) {
                if(!lowBucketToggle){
                    robot.output.outputHighBucketNoWait(3000);
                }else{
                    robot.output.outputLowBucketNoWait(3000);

                }
            }
            if (armsGamepad.wasLeftBumperPressed()) {
                robot.output.outputLoadNoWait(4000);
            }

            //HANG

            if(armsGamepad.wasOptionsPressed()){
                optionsPresses+=1;
                if(optionsPresses==1){
                    inHang = true;
                    robot.hangPhase1NoWait();
                }if(optionsPresses==2){
                    robot.hangPhase2V3();
                    hangManualControl=true;
                }
            }

            if(armsGamepad.wasBackPressed() && inHang){
                robot.hangPhase2Level2();
                hangManualControl=true;
            }

            if(hangManualControl){

                robot.drive.stopMotors();
                robot.hang.joystickDriveV2(gamepad1.left_stick_x, gamepad1.left_stick_y);
                telemetry.addLine("Hangleft: " + robot.hang.hang_Left.getCurrentPosition()+ " Hangright: "+ robot.hang.hang_Right.getCurrentPosition());
                robot.hangPhase2DelayedOps();

                //break out
                if(driverGamepad.wasHomePressed()){
                    hangManualControl=false;
                    optionsPresses=0;
                }
            }else{
                //drive
                robot.drive.universalDriveJoystickV2(
                        driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.left_stick_y,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,driverGamepad.gamepad.left_trigger > .5,
                        robot.drive.getHeadingODO());
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
