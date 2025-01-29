package pedroPathing;

import static java.lang.Thread.sleep;

import android.app.backup.BackupHelper;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class PedroBucket extends LinearOpMode {

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
            (WHEEL_DIAMETER_INCHES * 3.1415);final double FRONT_CLAW_OPENED = 0.1;
    final double FRONT_CLAW_CLOSED = 0.31;
    final double BACK_CLAW_OPENED = 0.1;
    final double BACK_CLAW_CLOSED = 0.33;
    final int SLIDES_SPECIMEN_DOWN = 0;
    final int SLIDES_SPECIMEN_PREP_HANG = 1450;
    final double FRONT_WRIST_HORIZONTAL = 0.61;
    final double STOPPER1_DOWN = 0.7;
    final double STOPPER2_DOWN = 0.74;    // offset seems slightly different on 2
    final double STOPPER1_UP = 0.0;
    final double STOPPER2_UP = 0.0;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Timer timer = new Timer();

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** robot poses **/
    private final Pose startPose = new Pose(9, 96, Math.toRadians(0));
    private final Pose bucketPose = new Pose(12.67, 125.9, Math.toRadians(-45));
    private final Pose s1Pose = new Pose(29, 118.1, Math.toRadians(0));
    private final Pose s2Pose = new Pose(29, 130.4, Math.toRadians(0));
    private final Pose s3Pose = new Pose(46.2, 127.9, Math.toRadians(90));
    private final Pose park = new Pose(72, 108, Math.toRadians(0));

    /** robot poses **/
    private PathChain bucket, s1, buckets1, s2, buckets2, s3, buckets3, toPark;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        bucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(bucketPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        s1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(s1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .build();

        buckets1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(s1Pose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-45))
                .build();

        s2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(s2Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .build();

        buckets2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(s2Pose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-45))
                .build();

        s3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(s2Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .build();

        buckets3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(s3Pose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-45))
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(park)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.3);
                follower.followPath(bucket, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    //slides up, drop into bucket
                    follower.followPath(s1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    //pick up sample, pass to back claw
                    follower.followPath(buckets1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    //slides up, drop into bucket
                    follower.followPath(s2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    //pick up sample, pass to back claw
                    follower.followPath(buckets2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    //slides up, drop into bucket
                    follower.followPath(s3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    //pick up sample, pass to back claw
                    follower.followPath(buckets3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    //go to park
                    follower.followPath(toPark,true);
                    setPathState(-1);
                }
                break;
        }

    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
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
        runtime.reset();
        opmodeTimer.resetTimer();
        setPathState(0);

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

    public void wallGrab(){
        while(backWrist.getPosition()!=0.14 && opModeIsActive()){
            backWrist.setPosition(0.14);
        }
        while(backClaw.getPosition()!=0.1 && opModeIsActive()){
            backClaw.setPosition(0.1);
        }
        while(slideR.getCurrentPosition() > 10 && opModeIsActive()){
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(0);
            slideL.setTargetPosition(0);
        }
        sleep(20);
        while(backClaw.getPosition()!=0.33 && opModeIsActive()){
            backClaw.setPosition(0.33);
        }
        while(slideR.getCurrentPosition() < SLIDES_SPECIMEN_PREP_HANG + 5 && opModeIsActive()){
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(SLIDES_SPECIMEN_PREP_HANG + 5);
            slideL.setTargetPosition(SLIDES_SPECIMEN_PREP_HANG + 5);
        }
    }

    public void specimenHang(){
        while (slideR.getCurrentPosition() > SLIDES_SPECIMEN_DOWN +5 && opModeIsActive()) {
            stopper1.setPosition(0);
            stopper2.setPosition(0);
            backWrist.setPosition(0.14);
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(SLIDES_SPECIMEN_DOWN);
            slideL.setTargetPosition(SLIDES_SPECIMEN_DOWN);
            if (slideR.getCurrentPosition() > 1000) {
                backClaw.setPosition(BACK_CLAW_CLOSED);
            }
            else{
                while(backClaw.getPosition()!=BACK_CLAW_OPENED && opModeIsActive()){
                    backClaw.setPosition(BACK_CLAW_OPENED);
                }
            }
        }
    }

    public void stoppersDown(){
        while(slideR.getCurrentPosition() < SLIDES_SPECIMEN_PREP_HANG + 5 && opModeIsActive()){
            stopper1.setPosition(STOPPER1_DOWN);
            stopper2.setPosition(STOPPER2_DOWN);
            backWrist.setPosition(0.14);
            backClaw.setPosition(BACK_CLAW_CLOSED);
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(SLIDES_SPECIMEN_PREP_HANG);
            slideL.setTargetPosition(SLIDES_SPECIMEN_PREP_HANG);
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

    public void ending(){
        wrist.setPosition(0);
        stopper1.setPosition(0);
        stopper2.setPosition(0);
        while(slideR.getCurrentPosition() > 0 && opModeIsActive()){
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(0);
            slideL.setTargetPosition(0);
        }
    }

    public void tongue(int pos) { //-1 is out, 1 is in
        tongue.setPosition(pos);
    }

    public void claw(double pos) { //0.2 is open, 0.55 is closed

        while (claw.getPosition() != pos && opModeIsActive()) {
            claw.setPosition(pos);
        }
    }

    public void wrist(double pos) { //0 is back, 0.62 is down for grab
        while (wrist.getPosition() != pos && opModeIsActive()) {
            wrist.setPosition(pos);
        }
    }

    public void backClaw(double pos){ //0 is closed, 0.35 is open
        while (backClaw.getPosition() != pos && opModeIsActive()) {
            backClaw.setPosition(pos);
        }
    }
    public void backWrist(double pos){ //0 is out, 0.65 is towards the other claw
        while (backWrist.getPosition() != pos && opModeIsActive()) {
            backWrist.setPosition(pos);
        }
    }
}