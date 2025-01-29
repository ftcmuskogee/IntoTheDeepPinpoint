package org.firstinspires.ftc.teamcode.OPModes;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.GoBilda.GoBildaPinpointDriver;

public class Mapp {
    boolean y = false;
    // names motors and sets motors to type null
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;
    public DcMotor leftFront = null;
    public DcMotor Arm = null;
    public DcMotor Arm2 = null;
    public Servo LittleArm = null;
    public Servo Elbow = null;
    public Servo Elbow2 = null;
    public Servo Claw = null;
    public Servo Wrist = null;
    public GoBildaPinpointDriverRR pinpoint; // pinpoint CH i2C port 0
    public IMU      imu              = null;





    //TODO: CHANGE THE VALUES OF THE CONSTRAINTS

    // Hardware Constants

    // Claw constraints
    public final double CLAW_OPEN = 0.5;
    public final double CLAW_CLOSE = 0.5;

    //Wrist constraints
    public final double WRIST_DOWN = 0.4;
    public final double WRIST_UP = 0;

    // Elbow constraints
    public final int ELBOW_GRAB_POSITION = 0;
    public final int ELBOW_HIGH_BAR_POSITION = 1150;

    // Arm constraints
    public final double ARM_DOWN_POWER = -0.2;
    public final double ARM_UP_POWER = 0.2;
    public final int ARM_RESET_POSITION = 0;
    public final int ARM_HIGH_BAR_POSITION = 1080;
    public final int ARM_LOW_BASKET_POSITION = 0;
    public final int ARM_DOWN_MAX = 1670;

    public final double ARM2_DOWN_POWER = -0.2;
    public final double ARM2_UP_POWER = 0.2;
    public final int ARM2_RESET_POSITION = 0;
    public final int ARM2_HIGH_BAR_POSITION = 1080;
    public final int ARM2_LOW_BASKET_POSITION = 0;
    public final int ARM2_DOWN_MAX = 1670;

    // sets hardware map to null and names it

    // creates runtime variable
    public ElapsedTime runtime = new ElapsedTime();
    HardwareMap Mapp = null;
    public void init(HardwareMap hmap, boolean teleopMode) {
        //sets up names for configuration
        Mapp = hmap;

        Elbow = hmap.get(Servo.class, "E");
        Elbow2 = hmap.get(Servo.class, "E2");
        LittleArm = hmap.get(Servo.class, "LA");
        Claw = hmap.get(Servo.class, "C");
        Wrist = hmap.get(Servo.class, "W");
        Arm = hmap.get(DcMotor.class, "ARM");
        Arm2 = hmap.get(DcMotor.class, "ARM2");

        rightFront = hmap.get(DcMotor.class, "RF");
        rightBack = hmap.get(DcMotor.class, "RB");
        leftBack = hmap.get(DcMotor.class, "LB");
        leftFront = hmap.get(DcMotor.class, "LF");

        imu  = Mapp.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        pinpoint = Mapp.get(GoBildaPinpointDriverRR.class,"pinpoint");
        pinpoint.resetPosAndIMU();



        Elbow.setPosition(100);
        Elbow2.setPosition(100);
        LittleArm.setPosition(0);
        Claw.setPosition(0);
        Wrist.setPosition(0);

        if (teleopMode) {

            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setPower(0);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setPower(0);
        Arm2.setTargetPosition(0);
        Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // function for driving forward
    //runs motors forward at 60% power
    public void Forward(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            leftFront.setPower(0.3);
            rightFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightBack.setPower(0.3);
        }
    }

    //function for driving backward
    //runs motors backward at 60% power
    public void Backward(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            leftFront.setPower(-0.6);
            rightFront.setPower(-0.6);
            leftBack.setPower(-0.6);
            rightBack.setPower(-0.6);
        }
    }

    // function for turning left
    //runs left motors backward at 60% power
    //runs right motors forward at 60% power
    public void Left(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            leftFront.setPower(-0.6);
            rightFront.setPower(0.6);
            leftBack.setPower(-0.6);
            rightBack.setPower(0.6);
        }
    }

    // function for turning right
    //runs right motors backward at 60% power
    //runs left motors forward at 60% power
    public void Right(double seconds) {
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)) {
            leftFront.setPower(0.6);
            rightFront.setPower(-0.6);
            leftBack.setPower(0.6);
            rightBack.setPower(-0.6);

        }
    }

    // function for strafing right
    //runs frontleft motor forward at 60% power
    //runs frontright motor backward at 60% power
    //runs backleft motor backward at 60% power
    //runs backright motor forward at 60% power
    public void RightStrafe(double seconds) {
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)) {
            leftFront.setPower(0.6);
            rightFront.setPower(-0.6);
            leftBack.setPower(-0.6);
            rightBack.setPower(0.6);

        }
    }

    // function for strafing left
    //runs frontleft motor backward at 60% power
    //runs frontright motor forward at 60% power
    //runs backleft motor forward at 60% power
    //runs backright motor backward at 60% power
    public void LeftStrafe(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            leftFront.setPower(-0.6);
            rightFront.setPower(0.6);
            leftBack.setPower(0.6);
            rightBack.setPower(-0.6);
        }
    }

    // function for turning off motors
    public void Off() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    //turns arm motor power off
    public void Aoff() {
        Arm.setPower(0);
    }

    //turns arm2 motor power off
    public void A2off() {Arm2.setPower(0);}

    //Elbow
    public void E(double position) {Elbow.setPosition(position);}

    //Elbow 2
    public void E2(double position) {Elbow2.setPosition(position);}

    //Little Arm
    public void LA(double position) {LittleArm.setPosition(position);}

    //Claw
    public void C(double position) {
        Claw.setPosition(position);
    }

    //Wrist
    public void W(double position) {
        Wrist.setPosition(position);
    }



    //move arm in
    public void IN(double seconds) {
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Arm.setPower(-.2);
            Arm2.setPower(-.2);
        }

    }
    //move arm out
    public void OUT(double seconds) {
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Arm.setPower(.2);
            Arm2.setPower(.2);
        }
    }
}
