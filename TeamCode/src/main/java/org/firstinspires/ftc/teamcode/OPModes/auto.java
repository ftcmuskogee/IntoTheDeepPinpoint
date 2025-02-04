package org.firstinspires.ftc.teamcode.OPModes;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;


import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.OPModes.Mapp;


//@Disabled
@Autonomous(name = "auto", group = "Autonomous Main")
public class auto extends LinearOpMode{


    public static String TEAM_NAME = "MIT";
    public static int TEAM_NUMBER = 4155;





    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_SAMPLES,
        BLUE_SPECIMENS,
        RED_SAMPLES,
        RED_SPECIMENS
    }

    public static START_POSITION startPosition;




    public LinearOpMode opMode = this;
    Mapp robot = new Mapp();



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Mapp robot = new Mapp();
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        selectStartingPosition();

        // Wait for the DS start button to be touched.
        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            runAutonoumousMode(drive);
        }
    }

    //end runOpMode();

    public void runAutonoumousMode(PinpointDrive drive) {


        //Initialize Pose2d as desired
//        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
        Pose2d specimenScoringPosition2 = new Pose2d(0, 0, 0);
        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
        Pose2d sampleScoringPosition2 = new Pose2d(0, 0, 0);
        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d pushSample = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
        Pose2d midwayPose3 = new Pose2d(0,0,0);
        Pose2d midwayPose4 = new Pose2d(0, 0,0);
        Pose2d midwayPose5 = new Pose2d(0,0,0);
        Pose2d midwayPose6 = new Pose2d(0,0,0);
        Pose2d midwayPose7 = new Pose2d(0,0,0);
        Pose2d midwayPose8 = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;

        switch (startPosition) {
            /*case BLUE_SAMPLES:
//                drive = new PinpointDrive(hardwareMap, initPose);
                sampleScoringPosition = new Pose2d(6, 34, Math.toRadians(238) );
                yellowSample1Position = new Pose2d(34, 27, Math.toRadians(0));
                yellowSample2Position = new Pose2d(34, 39, Math.toRadians(0));
                yellowSample3Position = new Pose2d(3, -10, Math.toRadians(0));
                midwayPose1 = new Pose2d(14, 30, Math.toRadians(180));
                midwayPose2 = new Pose2d(12, 8, Math.toRadians(180));
                midwayPose3 = new Pose2d(18, 20, Math.toRadians(0));
                midwayPose4 = new Pose2d(16, 25, Math.toRadians(90));
                midwayPose5 = new Pose2d(20, 20, Math.toRadians(75));
                midwayPose6 = new Pose2d(15, 20, Math.toRadians(135));
                midwayPose7 = new Pose2d(25, 20, Math.toRadians(135));
                midwayPose8 = new Pose2d(60, 20, Math.toRadians(135));
                parkPose = new Pose2d(50, -5, 270);
                break;*/

            case RED_SAMPLES:
  //              drive = new PinpointDrive(hardwareMap, initPose);
                sampleScoringPosition = new Pose2d(10, 10, Math.toRadians(135));
                sampleScoringPosition2 = new Pose2d(14, 10, Math.toRadians(135));
                yellowSample1Position = new Pose2d(23, 14 , Math.toRadians(0));
                yellowSample2Position = new Pose2d(23, 17, Math.toRadians(0));
                yellowSample3Position = new Pose2d(27, 15, Math.toRadians(45));
                midwayPose1 = new Pose2d(16, 17, Math.toRadians(0));
                midwayPose2 = new Pose2d(12, 8, Math.toRadians(180));
                midwayPose3 = new Pose2d(18, 20, Math.toRadians(0));
                parkPose = new Pose2d(50, -5, 270);
                break;

            /*case BLUE_SPECIMENS:
    //            drive = new PinpointDrive(hardwareMap, initPose);
                specimenScoringPosition = new Pose2d(23, 0, Math.toRadians(0));
                specimenScoringPosition2 = new Pose2d(21, 5, Math.toRadians(-45));
                grabSpecimenPosition = new Pose2d(8, -64, Math.toRadians(177));
                coloredSample1Position = new Pose2d(38, -56, 0);
                coloredSample2Position = new Pose2d(0, 0, 0);
                coloredSample3Position = new Pose2d(0, 0, 0);
                pushSample = new Pose2d(0, 0, 0);
                midwayPose1 = new Pose2d(18, -64, 0);
                midwayPose2 = new Pose2d(22, -64, Math.toRadians(177));
                midwayPose3 = new Pose2d(6, -64, Math.toRadians(185));
                midwayPose4 = new Pose2d(13, -34, Math.toRadians(-45));
                parkPose = new Pose2d(5, 15, -90);
                break;*/

            case RED_SPECIMENS:
      //          drive = new PinpointDrive(hardwareMap, initPose);
                specimenScoringPosition = new Pose2d(24.5, 16.4, Math.toRadians(0));
                specimenScoringPosition2 = new Pose2d(24.5, 20.4, Math.toRadians(0));
                grabSpecimenPosition = new Pose2d(8.1, -28.6, Math.toRadians(175));
                coloredSample1Position = new Pose2d(39, -32, 0);
                coloredSample2Position = new Pose2d(39, -35, 0);
                coloredSample3Position = new Pose2d(0, 0, 0);
                pushSample = new Pose2d(9, -23, 0);
                midwayPose1 = new Pose2d(50, -18, 0);
                midwayPose2 = new Pose2d(15, -29.2, Math.toRadians(175));
                midwayPose3 = new Pose2d(21, 16.4, Math.toRadians(0));
                midwayPose4 = new Pose2d(5, -10, Math.toRadians(0));
                midwayPose5 = new Pose2d(20, -28.6, Math.toRadians(175));
                midwayPose6 = new Pose2d(10, -28.6, Math.toRadians(175));
                midwayPose7 = new Pose2d(20.6, -6.3, Math.toRadians(175));
                parkPose = new Pose2d(5, -28.6, 0);
                break;

        }

        /**
         * For Sample Scoring into high bucket
         **/
        if (startPosition == START_POSITION.BLUE_SAMPLES ||
                startPosition == START_POSITION.RED_SAMPLES) {
            robot.init(hardwareMap);

            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
                            .build());

            // Raise Arm to high bucket scoring position
            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_GRAB);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_GRAB);
            safeWaitSeconds(.5);
            //close claw is wrist 1
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_CLOSE);
            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Elbow.setPosition(1);
            if(opModeIsActive()) robot.Elbow2.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_LOW_BASKET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM2_LOW_BASKET_POSITION);
            safeWaitSeconds(1);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_OPEN);
            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_RESET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM_RESET_POSITION);

            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_RESET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM_RESET_POSITION);
            safeWaitSeconds(.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(yellowSample2Position.position, yellowSample2Position.heading)
                            .build());

            // Raise Arm to high bucket scoring position
            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_GRAB);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_GRAB);
            safeWaitSeconds(.5);
            //close claw is wrist 1
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_CLOSE);
            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Elbow.setPosition(1);
            if(opModeIsActive()) robot.Elbow2.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_LOW_BASKET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM2_LOW_BASKET_POSITION);
            safeWaitSeconds(1);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_OPEN);
            safeWaitSeconds(.5);

            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_RESET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM_RESET_POSITION);
            safeWaitSeconds(.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample3Position.position, yellowSample3Position.heading)
                            .build());


            // Raise Arm to high bucket scoring position
            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_GRAB);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_GRAB);
            safeWaitSeconds(.5);
            //close claw is wrist 1
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_CLOSE);
            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Elbow.setPosition(1);
            if(opModeIsActive()) robot.Elbow2.setPosition(1);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition2.position, sampleScoringPosition2.heading)
                            .build());

            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_LOW_BASKET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM2_LOW_BASKET_POSITION);
            safeWaitSeconds(1);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_OPEN);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_RESET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM_RESET_POSITION);


        }   //end of if (startPosition == BLUE_SAMPLES || RED_SAMPLES)

        /**
         *  For Specimen Scoring onto high bar
         **/
        if (startPosition == START_POSITION.BLUE_SPECIMENS ||
                startPosition == START_POSITION.RED_SPECIMENS) {
            robot.init(hardwareMap);

            // Raise Arm to high bar scoring position

            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(coloredSample1Position.position, coloredSample1Position.heading)
                            .build());

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(pushSample.position, pushSample.heading)
                            .build());

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(coloredSample2Position.position, coloredSample2Position.heading)
                            .build());

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(pushSample.position, pushSample.heading)
                            .build());

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                    .build());

            safeWaitSeconds(1.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                            .build());

            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_GRAB);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_GRAB);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_CLOSE);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_UP);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_UP);
            //safeWaitSeconds(.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose3.position,midwayPose3.heading)
                            .build());

            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_HIGH_BAR_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM2_HIGH_BAR_POSITION);
            safeWaitSeconds(.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            if(opModeIsActive()) robot.Forward(.5);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_RESET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM_RESET_POSITION);
            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_OPEN);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(midwayPose5.position, midwayPose5.heading)
                    .build());

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                    .build());

            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_GRAB);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_GRAB);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_CLOSE);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Elbow.setPosition(robot.ELBOW_UP);
            if(opModeIsActive()) robot.Elbow2.setPosition(robot.ELBOW_UP);
            //safeWaitSeconds(.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose3.position,midwayPose3.heading)
                            .build());

            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_HIGH_BAR_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM2_HIGH_BAR_POSITION);
            safeWaitSeconds(.5);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition2.position, specimenScoringPosition2.heading)
                            .build());

            if(opModeIsActive()) robot.Forward(.5);
            safeWaitSeconds(.25);
            if(opModeIsActive()) robot.Arm.setPower(2);
            if(opModeIsActive()) robot.Arm2.setPower(2);
            if(opModeIsActive()) robot.Arm.setTargetPosition(robot.ARM_RESET_POSITION);
            if(opModeIsActive()) robot.Arm2.setTargetPosition(robot.ARM_RESET_POSITION);
            safeWaitSeconds(.5);
            if(opModeIsActive()) robot.Claw.setPosition(robot.CLAW_OPEN);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(midwayPose6.position, midwayPose6.heading)
                    .build());


        }   //end of if (startPosition == BLUE_SPECIMENS || RED_SPECIMENS)

    }



    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        Boolean menuActive = true;

        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (!isStopRequested() && menuActive) {
            telemetry.addData("Initializing Autonomous:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
            telemetry.addData("    Blue Yellow Samples   ", "(▢/X)");
            telemetry.addData("    Blue Specimens ", "(Δ/Y)");
            telemetry.addData("    Red Yellow Samples    ", "(O/B)");
            telemetry.addData("    Red Specimens  ", "(X/A)");
            /*if (gamepad1.x) {
                telemetry.addLine("Selecting BLUE_SAMPLES");
                startPosition = START_POSITION.BLUE_SAMPLES;
                menuActive = false;
            }
            if (gamepad1.y) {
                telemetry.addLine("Selecting BLUE_SPECIMEN");
                startPosition = START_POSITION.BLUE_SPECIMENS;
                menuActive = false;
            }*/
            if (gamepad1.b) {
                telemetry.addLine("Selecting RED_SAMPLES");
                startPosition = START_POSITION.RED_SAMPLES;
                menuActive = false;
            }
            if (gamepad1.a) {
                telemetry.addLine("Selecting RED_SPECIMEN");
                startPosition = START_POSITION.RED_SPECIMENS;
                menuActive = false;
            }
            telemetry.update();

        }
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

}   // end class