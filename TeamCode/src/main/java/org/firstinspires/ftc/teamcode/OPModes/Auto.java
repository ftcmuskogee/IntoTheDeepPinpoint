//package org.firstinspires.ftc.teamcode.OPModes;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//
//
//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
//import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
//import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
//
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.robot.Robot;
//
//
////import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
////import org.firstinspires.ftc.teamcode.hardware.HWProfile;
////import org.firstinspires.ftc.teamcode.libraries.RRMechOps;
//
//
////@Disabled
//@Autonomous(name = "auto", group = "Autonomous Main")
//public class auto extends LinearOpMode{
//
//    public static String TEAM_NAME = "MIT";
//    public static int TEAM_NUMBER = 4155;
//
//
//
//    //Define and declare Robot Starting Locations
//    public enum START_POSITION {
//        BLUE_SAMPLES,
//        BLUE_SPECIMENS,
//        RED_SAMPLES,
//        RED_SPECIMENS
//    }
//
//    public static START_POSITION startPosition;
//
//    //public final static HWProfile robot = new HWProfile();
//    public LinearOpMode opMode = this;
//    //    public CSAutoParams params = new CSAutoParams();
////    public RRMechOps mechOps = new RRMechOps(robot, opMode, params);
//    Mapp robot = new Mapp();
//
//
//    public double startDelay = 0;
//    public int delayPosition = 0;
//
//    public DcMotor rightFront = null;
//    public DcMotor rightBack = null;
//    public DcMotor leftBack = null;
//    public DcMotor leftFront = null;
//    public Servo Claw = null;
//    public DcMotor Elbow = null;
//    public Servo Wrist = null;
//    public DcMotor Arm = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        Mapp robot = new Mapp();
//        robot.init(hardwareMap, false);
//        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//
//
//        //Key Pay inputs to selecting Starting Position of robot
//        selectStartingPosition();
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("Selected Starting Position", startPosition);
//        telemetry.addData("Selected Delay Position", delayPosition);
//        telemetry.addData("Selected Delay Position", startDelay);
//        telemetry.addLine("Open CV Vision for Red/Blue Team Element Detection");
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addLine("The starting point of the robot is assumed to be on the starting tile, " +
//                "and along the edge farther from the truss legs. ");
//        telemetry.addLine("You should also have a webcam connected and positioned in a way to see " +
//                "the middle spike mark and the spike mark away from the truss (and ideally nothing else). " +
//                "We assumed the camera to be in the center of the robot. ");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            telemetry.addData("Selected Starting Position", startPosition);
//            telemetry.addData("Selected Delay Position", delayPosition);
//            telemetry.addData("Selected Delay Position", startDelay);
//
//        }
//
//        //Game Play Button  is pressed
//        if (opModeIsActive() && !isStopRequested()) {
//            runAutonoumousMode();
//        }
//    }
//
//    //end runOpMode();
//
//    public void runAutonoumousMode() {
//        robot.init(hardwareMap, false);
//        Mapp robot = new Mapp();
//
//
//        //Initialize Pose2d as desired
//        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
//        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
//        Pose2d specimenScoringPosition2 = new Pose2d(0, 0, 0);
//        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
//        Pose2d sampleScoringPosition2 = new Pose2d(0, 0, 0);
//        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
//        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
//        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
//        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
//        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
//        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
//        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
//        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose3 = new Pose2d(0,0,0);
//        Pose2d midwayPose4 = new Pose2d(0, 0,0);
//        Pose2d midwayPose5 = new Pose2d(0,0,0);
//        Pose2d midwayPose6 = new Pose2d(0,0,0);
//        Pose2d midwayPose7 = new Pose2d(0,0,0);
//        Pose2d midwayPose8 = new Pose2d(0,0,0);
//        Pose2d parkPose = new Pose2d(0, 0, 0);
//        double waitSecondsBeforeDrop = 0;
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
//
//        switch (startPosition) {
//            case BLUE_SAMPLES:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                sampleScoringPosition = new Pose2d(6, 34, Math.toRadians(238) );
//                yellowSample1Position = new Pose2d(34, 27, Math.toRadians(0));
//                yellowSample2Position = new Pose2d(34, 39, Math.toRadians(0));
//                yellowSample3Position = new Pose2d(3, -10, Math.toRadians(0));
//                midwayPose1 = new Pose2d(14, -7, Math.toRadians(180));
//                midwayPose2 = new Pose2d(12, 8, Math.toRadians(180));
//                midwayPose3 = new Pose2d(18, 20, Math.toRadians(0));
//                midwayPose4 = new Pose2d(16, 25, Math.toRadians(90));
//                midwayPose5 = new Pose2d(20, 20, Math.toRadians(75));
//                midwayPose6 = new Pose2d(15, 20, Math.toRadians(135));
//                midwayPose7 = new Pose2d(25, 20, Math.toRadians(135));
//                midwayPose8 = new Pose2d(60, 20, Math.toRadians(135));
//                parkPose = new Pose2d(50, -5, 270);
//                break;
//
//            case RED_SAMPLES:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                sampleScoringPosition = new Pose2d(4, 28, Math.toRadians(238) );
//                sampleScoringPosition2 = new Pose2d(6, 34, Math.toRadians(238) );
//                yellowSample1Position = new Pose2d(34, 27, Math.toRadians(0));
//                yellowSample2Position = new Pose2d(34, 39, Math.toRadians(0));
//                yellowSample3Position = new Pose2d(3, -10, Math.toRadians(0));
//                midwayPose1 = new Pose2d(14, -7, Math.toRadians(180));
//                midwayPose2 = new Pose2d(12, 8, Math.toRadians(180));
//                midwayPose3 = new Pose2d(18, 20, Math.toRadians(0));
//                midwayPose4 = new Pose2d(16, 25, Math.toRadians(90));
//                midwayPose5 = new Pose2d(20, 20, Math.toRadians(75));
//                midwayPose6 = new Pose2d(15, 20, Math.toRadians(135));
//                midwayPose7 = new Pose2d(25, 20, Math.toRadians(135));
//                midwayPose8 = new Pose2d(60, 20, Math.toRadians(135));
//                parkPose = new Pose2d(50, -5, 270);
//                break;
//
//            case BLUE_SPECIMENS:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                specimenScoringPosition = new Pose2d(23, 0, Math.toRadians(0));
//                specimenScoringPosition2 = new Pose2d(21, 5, Math.toRadians(-45));
//                grabSpecimenPosition = new Pose2d(8, -64, Math.toRadians(177));
//                coloredSample1Position = new Pose2d(38, -56, 0);
//                coloredSample2Position = new Pose2d(0, 0, 0);
//                coloredSample3Position = new Pose2d(0, 0, 0);
//                midwayPose1 = new Pose2d(18, -64, 0);
//                midwayPose2 = new Pose2d(22, -64, Math.toRadians(177));
//                midwayPose3 = new Pose2d(6, -64, Math.toRadians(185));
//                midwayPose4 = new Pose2d(13, -34, Math.toRadians(-45));
//                parkPose = new Pose2d(5, 15, -90);
//                break;
//
//            case RED_SPECIMENS:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                drive = new MecanumDrive(hardwareMap, initPose);
//                specimenScoringPosition = new Pose2d(22, 0, Math.toRadians(0));
//                specimenScoringPosition2 = new Pose2d(21, 5, Math.toRadians(-45));
//                grabSpecimenPosition = new Pose2d(5, -64, Math.toRadians(177));
//                coloredSample1Position = new Pose2d(34, -63, 0);
//                coloredSample2Position = new Pose2d(0, 0, 0);
//                coloredSample3Position = new Pose2d(0, 0, 0);
//                midwayPose1 = new Pose2d(18, -64, 0);
//                midwayPose2 = new Pose2d(22, -64, Math.toRadians(177));
//                midwayPose3 = new Pose2d(3, -64, Math.toRadians(185));
//                midwayPose4 = new Pose2d(13, -34, Math.toRadians(-45));
//                parkPose = new Pose2d(5, 15, -90);
//                break;
//
//        }
//
//        /**
//         * For Sample Scoring into high bucket
//         **/
//        if (startPosition == START_POSITION.BLUE_SAMPLES ||
//                startPosition == START_POSITION.RED_SAMPLES) {
//            robot.init(hardwareMap, false);
//
//            robot.W(1);
//
//            // Drive to scoring position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
//                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
//                            .build());
//
//            // Raise Arm to high bucket scoring position
//            if(opModeIsActive()) {
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(850);
//                robot.W(.6);
//                sleep(200);
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(1600);
//                sleep(800);
//
//
//
//                // Release the sample into the bucket
//                // Lower the arm
//                if(opModeIsActive()) {
//                    robot.W(1);
//                    sleep(500);
//                    robot.C(0.1);
//                    sleep(200);
//                    robot.W(.6);
//                    sleep(800);
//
//
//                }
//                //bring arm back in
//                if(opModeIsActive()) {
//                    robot.Arm.setPower(2);
//                    robot.Arm.setTargetPosition(0);
//                    sleep(800);
//                    robot.Elbow.setPower(2);
//                    robot.Elbow.setTargetPosition(-500);
//                    sleep(800);
//                    robot.Claw.setPosition(.1);
//                    robot.Right(.67);
//
//                }
//
//                // TODO: Add code to release the sample and lower the arm
//            }
//
//            // Drive to Sample1 Position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
//                            .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
//                            .build());
//
//            //Pick up Sample1
//            if(opModeIsActive()) {
//                robot.W(1);
//                sleep(800);
//                robot.C(-0.3);
//                sleep(500);
//                robot.W(.6);
//
//
//            }
//
//            // Drive to scoring position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
//                            .build());
//
//            // Release the yellowsample1 into the bucket
//            // Lower the arm
//            if(opModeIsActive()) {
//                robot.Left(.57);
//            }
//
//            //
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(sampleScoringPosition2.position, sampleScoringPosition2.heading)
//                            .build());
//
//            if(opModeIsActive()) {
//                sleep(800);
//                robot.W(.6);
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(950);
//                sleep(500);
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(2000);
//                sleep(800);}
//
//            if(opModeIsActive()) {
//                robot.W(1);
//                sleep(500);
//                robot.C(0.1);
//                sleep(200);
//                robot.W(.6);
//                sleep(800);}
//
//            if(opModeIsActive()) {
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(0);
//                sleep(1000);
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(-500);
//                sleep(1000);
//                robot.Claw.setPosition(.1);
//                robot.Right(.72);
//                robot.W(1);
//
//            }
//
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
//                            .build());
//
//        }   //end of if (startPosition == BLUE_SAMPLES || RED_SAMPLES)
//
//        /**
//         *  For Specimen Scoring onto high bar
//         **/
//        if (startPosition == START_POSITION.BLUE_SPECIMENS ||
//                startPosition == START_POSITION.RED_SPECIMENS) {
//            robot.init(hardwareMap, false);
//
//            // Raise Arm to high bar scoring position
//
//            // Drive to scoring position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
//                            .build());
//
//            if(opModeIsActive()){
//                //move arm up to score
//                robot.C(-0.3);
//                sleep(500);
//                robot.W(.6);
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(850);
//                sleep(1000);
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(1300);
//                robot.C(-0.3);
//                sleep(1500);
//                //put specimen on bar
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(150);
//                sleep(1000);
//                robot.C(0.1);
//                sleep(1000);
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(0);
//            }
//
//            // Drive to color sample1 Position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                            .build());
//
//            //move all back to reset position
//            if(opModeIsActive()){
//                robot.C(.1);
//                sleep(500);
//                robot.W(1);
//                robot.Elbow.setPower(-2);
//                robot.Elbow.setTargetPosition(-500);
//                sleep(500);
//            }
//
//
//            //drive to pick up sample
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(coloredSample1Position.position, coloredSample1Position.heading)
//                            .build());
//
//            //pick up sample
//            if(opModeIsActive()){
//                robot.C(-0.3);
//                sleep(500);
//            }
//
//            //drive to midway pose and turn around
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                            .build());
//
//            //turn around w/ sample
//            if(opModeIsActive()){
//                robot.Right(.75);
//            }
//
//            //turn with specimen
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
//                            .build());
//
//            if(opModeIsActive()){
//                robot.C(.1);
//            }
//
//            if(opModeIsActive()){
//                robot.Right(0.12);
//            }
//
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
//                            .build());
//
//            //wait for specimen
//            if(opModeIsActive()){
//                sleep(2000);
//            }
//
//            if(opModeIsActive()){
//                robot.Elbow.setPower(-2);
//                robot.Elbow.setTargetPosition(-500);
//                robot.Arm.setPower(-2);
//                robot.Arm.setTargetPosition(-500);
//
//
//            }
//
//            /*Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
//                            .build());
//
//            //grab specimen from wall
//            if(opModeIsActive()){
//                robot.W(.6);
//                robot.C(.1);
//                sleep(500);
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(0);
//                sleep(500);
//                robot.C(-0.3);
//                sleep(500);
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(390);
//                sleep(500);
//                robot.Right(.15);
//                sleep(500);
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(0);
//
//            }
//
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
//                            .strafeToLinearHeading(specimenScoringPosition2.position, specimenScoringPosition2.heading)
//                            .build());
//
//            if(opModeIsActive()){
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(850);
//                sleep(1000);
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(1300);
//                sleep(1500);
//                //put specimen on bar
//                robot.Elbow.setPower(2);
//                robot.Elbow.setTargetPosition(150);
//                sleep(1000);
//                robot.C(0.1);
//                sleep(1000);
//                robot.Arm.setPower(2);
//                robot.Arm.setTargetPosition(0);
//            }
//*/
//        }   //end of if (startPosition == BLUE_SPECIMENS || RED_SPECIMENS)
//
//    }
//
//
//
//
//
//    //Method to select starting position using X, Y, A, B buttons on gamepad
//    public void selectStartingPosition() {
//        State setupConfig = State.START_POSITION;
//        Boolean menuActive = true;
//
//        telemetry.setAutoClear(true);
//        telemetry.clearAll();
//        //******select start pose*****
//        while(!isStopRequested() && menuActive){
//            switch(setupConfig){
//                case START_POSITION:
//                    telemetry.addData("Initializing Autonomous:",
//                            TEAM_NAME, " ", TEAM_NUMBER);
//                    telemetry.addData("---------------------------------------","");
//                    telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
//                    telemetry.addData("    Blue Yellow Samples   ", "(▢/X)");
//                    telemetry.addData("    Blue Specimens ", "(Δ/Y)");
//                    telemetry.addData("    Red Yellow Samples    ", "(O/B)");
//                    telemetry.addData("    Red Specimens  ", "(X/A)");
//                    if(gamepad1.x){
//                        startPosition = START_POSITION.BLUE_SAMPLES;
//                        menuActive = false;
//                    }
//                    if(gamepad1.y){
//                        startPosition = START_POSITION.BLUE_SPECIMENS;
//                        menuActive = false;
//                    }
//                    if(gamepad1.b){
//                        startPosition = START_POSITION.RED_SAMPLES;
//                        menuActive = false;
//                    }
//                    if(gamepad1.a){
//                        startPosition = START_POSITION.RED_SPECIMENS;
//                        menuActive = false;
//                    }
//                    telemetry.update();
//                    break;
//
//            }
//            telemetry.update();
//        }
//        telemetry.clearAll();
//    }
//
//    //method to wait safely with stop button working if needed. Use this instead of sleep
//    public void safeWaitSeconds(double time) {
//        ElapsedTime timer = new ElapsedTime(SECONDS);
//        timer.reset();
//        while (!isStopRequested() && timer.time() < time) {
//        }
//    }
//
//    public enum State {
//        START_POSITION,
//        PARK_POSITION
//    }
//
//}   // end class