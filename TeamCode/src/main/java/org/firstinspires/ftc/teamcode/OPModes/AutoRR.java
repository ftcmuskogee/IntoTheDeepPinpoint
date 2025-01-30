//package org.firstinspires.ftc.teamcode.OPModes;
//
//import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//
//
////@Disabled
//@Autonomous(name = "AutoRR", group = "Autonomous Main")
//public class AutoRR extends LinearOpMode{
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
//    public LinearOpMode opMode = this;
//    Mapp robot = new Mapp();
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
//        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//
//        //Key Pay inputs to selecting Starting Position of robot
//        selectStartingPosition();
//
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
//        //Game Play Button  is pressed
//        if (opModeIsActive() && !isStopRequested()) {
//            runAutonoumousMode();
//        }
//    }
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
//        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);
//
//        switch (startPosition) {
//            case BLUE_SAMPLES:
//                drive = new PinpointDrive(hardwareMap, initPose);
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
//                drive = new PinpointDrive(hardwareMap, initPose);
//                sampleScoringPosition = new Pose2d(1.4, 12.7, Math.toRadians(133.6) );
//                yellowSample1Position = new Pose2d(24, 15, Math.toRadians(0));
//                yellowSample2Position = new Pose2d(38, 10, Math.toRadians(0));
//                yellowSample3Position = new Pose2d(40, 10, Math.toRadians(22));
//                midwayPose1 = new Pose2d(14, -7, Math.toRadians(180));
//                midwayPose2 = new Pose2d(12, 8, Math.toRadians(180));
//                midwayPose3 = new Pose2d(18, 20, Math.toRadians(0));
//                parkPose = new Pose2d(50, -5, 270);
//                break;
//
//            case BLUE_SPECIMENS:
//                drive = new PinpointDrive(hardwareMap, initPose);
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
//                drive = new PinpointDrive(hardwareMap, initPose);
//                drive = new PinpointDrive(hardwareMap, initPose);
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
//            // Drive to scoring position
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
//                            .build());
//
//            //move arm down and grab sample
//            if (opModeIsActive()) robot.W(60);
//            if (opModeIsActive()) robot.E(10);
//            if (opModeIsActive()) robot.E2(10);
//            if (opModeIsActive()) robot.C(0);
//
//            //move arm up
//            if (opModeIsActive()) robot.E(100);
//            if (opModeIsActive()) robot.E2(100);
//
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
//
//            }
//
//
//            }
//
//        }   //end of if (startPosition == BLUE_SPECIMENS || RED_SPECIMENS)
//
//
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