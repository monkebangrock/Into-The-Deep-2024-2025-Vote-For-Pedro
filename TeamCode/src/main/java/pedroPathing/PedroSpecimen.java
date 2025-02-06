package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous
public class PedroSpecimen extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotorEx slideR = null;
    private DcMotorEx slideL = null;
    private DcMotorEx armHinge = null;
    private Servo tongue;
    private Servo claw;
    private Servo wrist;
    private Servo backWrist;
    private Servo backClaw;
    private Servo rotWrist;
    private Servo stopper1;
    private Servo stopper2;
    int slideTarget;
    boolean slideMoving;
    int slideLevel;
    boolean slideInput;
    int armTarget;
    boolean armMoving;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;

    //constants
    final double FRONT_CLAW_OPENED = 0.1;
    final double FRONT_CLAW_CLOSED = 0.31;
    final double BACK_CLAW_OPENED = 0.1;
    final double BACK_CLAW_CLOSED = 0.33;
    final int ARM_POS_UP = -180;
    final int ARM_POS_DOWN = -750;
    final int ARM_POS_TILT = -1310;
    final int SLIDES_BUCKET_DOWN = 0;
    final int SLIDES_BUCKET_LOW = 1730;
    final int SLIDES_BUCKET_HIGH = 3100;
    final int SLIDES_SPECIMEN_DOWN = 100;
    final int SLIDES_SPECIMEN_TRANSFER = 640;
    final int SLIDES_SPECIMEN_PREP_HANG = 1450;
    final int SLIDES_ROBOT_HANG = 1450;
    final double FRONT_WRIST_HORIZONTAL = 0.61;
    final double STOPPER1_DOWN = 0.7;
    final double STOPPER2_DOWN = 0.74;    // offset seems slightly different on 2
    final double STOPPER1_UP = 0.0;
    final double STOPPER2_UP = 0.0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;
    private Timer timer = new Timer();

    double tonguePos = 0.0;
    double rotWristPos = FRONT_WRIST_HORIZONTAL;
    boolean grabbing = false;
    boolean depositMode = false;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /**
     * robot poses
     **/
    private final Pose startPose = new Pose(9, 72, Math.toRadians(180));
    private final Pose hangPose = new Pose(36.2, 72, Math.toRadians(180));
    private final Pose s1Pose = new Pose(68, 27, Math.toRadians(180));
    private final Pose s1CP1 = new Pose(14, 13);
    private final Pose s1CP2 = new Pose(62, 45);
    private final Pose corner = new Pose(10, 24, Math.toRadians(180));
    private final Pose s2Pose = new Pose(65, 11, Math.toRadians(180));
    private final Pose s2CP1 = new Pose(62, 34);
    private final Pose betweenCP = new Pose(21, 55);
    /**
     * robot poses
     **/
    private PathChain hang1, s1, s1Push, s2, s2Push, hang2, toCorner;

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
    public void buildPaths() {
        hang1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(hangPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        s1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose), new Point(s1CP1), new Point(s1CP2), new Point(s1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        s1Push = follower.pathBuilder()
                .addPath(new BezierLine(new Point(s1Pose), new Point(corner)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        s2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(corner), new Point(s2CP1), new Point(s2Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        s2Push = follower.pathBuilder()
                .addPath(new BezierLine(new Point(s2Pose), new Point(corner)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        hang2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(hangPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        toCorner = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose), new Point(betweenCP), new Point(corner)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.3);
                follower.followPath(hang1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //hang
                    follower.followPath(s1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(s1Push);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(s2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(s2Push);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    //pick up specimen
                    follower.followPath(hang2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //hang
                    follower.followPath(toCorner);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    //pickup
                    follower.followPath(hang2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //hang, then park
                    follower.followPath(toCorner);
                    ending();
                    setPathState(-1);
                }
                break;
        }

    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");
        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        armHinge = hardwareMap.get(DcMotorEx.class, "armHinge");
        tongue = hardwareMap.get(Servo.class, "tongue");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        backWrist = hardwareMap.get(Servo.class, "backWrist");
        backClaw = hardwareMap.get(Servo.class, "backClaw");
        rotWrist = hardwareMap.get(Servo.class, "rotWrist");
        stopper1 = hardwareMap.get(Servo.class, "stopper1");
        stopper2 = hardwareMap.get(Servo.class, "stopper2");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        slideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armHinge.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //brake motors
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide stuff
        slideR.setTargetPosition(0);
        slideL.setTargetPosition(0);
        armHinge.setTargetPosition(0);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armHinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideTarget = 0;
        slideLevel = 0;
        armTarget = 0;

        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        armHinge.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        backWrist.setDirection(Servo.Direction.REVERSE);
        tongue.setDirection(Servo.Direction.REVERSE);
        stopper1.setDirection(Servo.Direction.FORWARD);
        stopper2.setDirection(Servo.Direction.REVERSE);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Wait for the game to start (driver presses START)
        waitForStart();
        waitForStart();
        runtime.reset();
        wrist.setPosition(0.48);
        claw.setPosition(FRONT_CLAW_OPENED);
        backWrist.setPosition(0.14);
        backClaw.setPosition(BACK_CLAW_CLOSED);
        rotWrist.setPosition(rotWristPos);
        stopper1.setPosition(STOPPER1_UP);
        stopper2.setPosition(STOPPER2_UP);
        tongue.setPosition(0);
        imu.resetYaw();

        slideR.setTargetPosition(0);
        slideL.setTargetPosition(0);
        armHinge.setTargetPosition(0);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armHinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideTarget = 0;
        slideMoving = false;
        slideLevel = 0;
        slideInput = false;
        armTarget = 0;
        armMoving = false;
        depositMode = false;
        grabbing = false;
        rotWristPos = FRONT_WRIST_HORIZONTAL;
        tonguePos = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void slide(int up) {
        slideMoving = true;
        slideInput = true;
        slideR.setMotorEnable();
        slideL.setMotorEnable();
        if (up == 1) {
            slideTarget = SLIDES_BUCKET_HIGH;
            slideR.setTargetPosition(slideTarget);
            slideL.setTargetPosition(slideTarget);
            slideLevel = 2;
            telemetry.addData("Level:", "2");
            telemetry.update();
        }
        else if (up == 0) {
            slideTarget = SLIDES_BUCKET_DOWN;
            slideR.setTargetPosition(slideTarget);
            slideL.setTargetPosition(slideTarget);
            slideLevel = 0;
            telemetry.addData("Level:", "0");
            telemetry.update();
        }
    }

    public void arm(int ticks) {
        armHinge.setVelocity(1000);
        armHinge.setTargetPosition(ticks);
        if (ticks > armHinge.getCurrentPosition()) {
            while (armHinge.getCurrentPosition() < ticks && opModeIsActive()) {
                armHinge.setVelocity(1000);
                armHinge.setTargetPosition(ticks);
            }
        } else {
            while (armHinge.getCurrentPosition() > ticks && opModeIsActive()) {
                armHinge.setVelocity(1000);
                armHinge.setTargetPosition(ticks);
            }
        }
        armMoving = true;
        telemetry.addData("arm:", armHinge.getCurrentPosition());
        telemetry.addData("tgt pos:", armHinge.getTargetPosition());
        telemetry.addData("gp:", gamepad2.left_stick_y);
        telemetry.update();
    }

    public void grab() {
        tongue.setPosition(0);
        claw.setPosition(0.1);
        wrist.setPosition(0.7);
        armTarget = -810;
        int curPos = armHinge.getCurrentPosition();
        while (curPos >= -810 && opModeIsActive()) {
            armHinge.setMotorEnable();
            armHinge.setVelocity(800);
            armHinge.setTargetPosition(-810);
            armMoving = true;
            curPos = armHinge.getCurrentPosition();
        }
        sleep(200);
        claw.setPosition(0.32);
    }

    public void grab2() {
        tongue.setPosition(0);
        claw.setPosition(0.1);
        wrist.setPosition(0.7);
        rotWrist.setPosition(0.11);
        armTarget = -810;
        int curPos = armHinge.getCurrentPosition();
        while (curPos >= -810 && opModeIsActive()) {
            armHinge.setMotorEnable();
            armHinge.setVelocity(800);
            armHinge.setTargetPosition(-810);
            armMoving = true;
            curPos = armHinge.getCurrentPosition();
        }
        sleep(200);
        claw.setPosition(0.32);
    }

    public void claw(double pos) { //0.2 is open, 0.55 is closed

        while (claw.getPosition() != pos && opModeIsActive()) {
            claw.setPosition(pos);
        }
    }

    public void wrist(double pos) { //0 is back, 0.15 is in line w/ arm
        while (wrist.getPosition() != pos && opModeIsActive()) {
            wrist.setPosition(pos);
        }
    }

    public void raiseSlides() {
        while (slideR.getCurrentPosition() < 3100 && opModeIsActive()) {
            backWrist.setPosition(0.14);
            slideR.setVelocity(5000);
            slideL.setVelocity(5000);
            slideR.setTargetPosition(3100);
            slideL.setTargetPosition(3100);
            if (slideR.getCurrentPosition() > 3100) {
                backClaw.setPosition(BACK_CLAW_CLOSED);
            } else {
                while (backClaw.getPosition() != BACK_CLAW_OPENED && opModeIsActive()) {
                    backClaw.setPosition(BACK_CLAW_OPENED);
                }
            }
        }
    }

    public void transfer() { //fix
        try {
            //motor first
            backClaw.setPosition(BACK_CLAW_CLOSED);
            int target = SLIDES_SPECIMEN_TRANSFER;
            backWrist.setPosition(0.77);
            rotWrist.setPosition(FRONT_WRIST_HORIZONTAL);
            rotWristPos = FRONT_WRIST_HORIZONTAL;
            wrist.setPosition(0.04);
            tongue.setPosition(0);
            tonguePos = 0;
            while ((slideR.getCurrentPosition() > (target + 5) || slideR.getCurrentPosition() < (target - 5)) && opModeIsActive()) {
                slideR.setVelocity(1000);
                slideL.setVelocity(1000);
                slideR.setTargetPosition(target);
                slideL.setTargetPosition(target);
                slideLevel = 0;
                telemetry.addData("SlideR Pos", slideR.getCurrentPosition());
                telemetry.addData("SlideR Tgt", slideR.getTargetPosition());
                telemetry.update();
            }
            backClaw.setPosition(BACK_CLAW_OPENED);
            armTarget = ARM_POS_UP;
            int curPos = armHinge.getCurrentPosition();
            while ((curPos < (ARM_POS_UP - 1) || curPos > (ARM_POS_UP + 1)) && opModeIsActive()) {
                armHinge.setMotorEnable();
                if(curPos > -300){
                    armHinge.setVelocity(300);
                }
                else if(curPos < -400){
                    armHinge.setVelocity(1500);
                }
                armHinge.setVelocity(800);
                armHinge.setTargetPosition(ARM_POS_UP);
                armMoving = true;
                curPos = armHinge.getCurrentPosition();
            }
            backClaw.setPosition(BACK_CLAW_CLOSED);
            sleep(300);
            claw.setPosition(FRONT_CLAW_OPENED);
            sleep(200);
            backWrist.setPosition(0.14);
            depositMode = false;
            // get slide in prep position
            slideR.setVelocity(5000);
            slideL.setVelocity(5000);
            slideR.setTargetPosition(slideTarget);
            slideL.setTargetPosition(slideTarget);
            slideLevel = 1;
            grabbing = false;
        } catch (Exception ex) {

        }
    }

    public void backClaw(int state){
        if(state == 0)
            backClaw.setPosition(BACK_CLAW_CLOSED);
        else
            backClaw.setPosition(BACK_CLAW_OPENED);
    }

    public void ending() {
        wrist.setPosition(0);
        stopper1.setPosition(0);
        stopper2.setPosition(0);
        while (slideR.getCurrentPosition() > 0 && opModeIsActive()) {
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(0);
            slideL.setTargetPosition(0);
        }
    }
}

