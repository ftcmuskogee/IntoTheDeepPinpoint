package org.firstinspires.ftc.teamcode.OPModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Locale;


@TeleOp(name="Teleopp")

public class Teleopp extends LinearOpMode {



    Mapp robot = new Mapp();


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odo.recalibrateIMU();
        odo.resetPosAndIMU();



        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();




        double speed;
        double extend;

        int armPosition = robot.ARM_RESET_POSITION;
        int arm2Position = robot.ARM_RESET_POSITION;




        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));


        waitForStart();
        //elbowExtendTime.reset();

        while (opModeIsActive()) {



            odo.update();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));



            //sets all 4 base motors to the left and right joysticks on gamepad 1
            //uses the variables from SampleMecanumDrive to adjust motors
            //left stick in the y direction is for going forward and backward at 80% power
            //left stick in the x direction is for strafing left and right at 80% power
            //right stick in the x direction is for turning left and right at 80% power
            //was x: y
            //was y: x
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y, -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            robot.Arm.setTargetPosition((int) armPosition);
            robot.Arm2.setTargetPosition((int) arm2Position);
            robot.Arm.setPower(2);
            robot.Arm2.setPower(2);
            //move arm
            if(gamepad2.right_stick_y > 0.05){
                armPosition = armPosition - 20;
                arm2Position = arm2Position + 20;
            }
            if (gamepad2.right_stick_y < -0.05){
                armPosition = armPosition + 20;
                arm2Position = arm2Position - 20;
            }


            //wrist horizontal
            if (gamepad2.right_trigger > 0) {
                robot.Wrist.setPosition(1);
            }
            // wrist vertical
            if (gamepad2.left_trigger > 0) {
                robot.Wrist.setPosition(0);
            }

            //close claw
            if (gamepad2.left_bumper) {
                robot.Claw.setPosition(0);
                //open claw
            } else if (gamepad2.right_bumper) {
                robot.Claw.setPosition(1);
            }

            if (gamepad2.dpad_up) {
                robot.Elbow.setPosition(1);
                robot.Elbow2.setPosition(1);
            } else if (gamepad2.dpad_down) {
                robot.Elbow.setPosition(.1);
                robot.Elbow2.setPosition(.1);
            }

            //arm out
            if (gamepad2.dpad_left) {
                robot.LittleArm.setPosition(1);
                //arm in
            } else if (gamepad2.dpad_right) {
                robot.LittleArm.setPosition(0);
            }

            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Position", data);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("armPosition", armPosition);
            telemetry.addData("arm2Position", arm2Position);
            telemetry.addData("Arm Encoder Value", robot.Arm.getCurrentPosition());
            telemetry.addData("Arm Encoder Value", robot.Arm2.getCurrentPosition());
            telemetry.update();

        }
    }
}




