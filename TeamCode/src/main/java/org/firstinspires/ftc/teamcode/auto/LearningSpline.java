package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;

@Autonomous
public class LearningSpline extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory trajectory1 = sampleMecanumDrive.trajectoryBuilder(startPose,false)
                .splineToConstantHeading(new Vector2d(15,15),Math.toRadians(0))
                .build();
        Trajectory trajectory2 = sampleMecanumDrive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .strafeLeft(15)
                .build();
        waitForStart();


        if(isStopRequested()) return;
        sampleMecanumDrive.followTrajectory(trajectory1);
        sampleMecanumDrive.turn(Math.toRadians(90));
        sampleMecanumDrive.followTrajectory(trajectory2);
    }
}
