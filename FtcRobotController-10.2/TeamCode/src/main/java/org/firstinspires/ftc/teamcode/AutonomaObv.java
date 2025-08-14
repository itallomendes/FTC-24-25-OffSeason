package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutonomaObv extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Garras garra = new Garras(hardwareMap);
        Elevador elevador = new Elevador(hardwareMap);
        SistemasDoRobo sistemas = new SistemasDoRobo(hardwareMap, true, true);
        SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

        Pose2d posicaoInicial = new Pose2d(-9.2,62, Math.toRadians(-90));
        chassi.setPoseEstimate(posicaoInicial);


        Trajectory InitialScore = chassi.trajectoryBuilder(posicaoInicial, true)
                .lineToLinearHeading(new Pose2d(-5, 29, Math.toRadians(-90)))
                .build();
        Trajectory ColetaSamples = chassi.trajectoryBuilder(InitialScore.end(), true)
                .lineToLinearHeading(new Pose2d(-5, 40, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples1 = chassi.trajectoryBuilder(ColetaSamples.end(), true)
                .lineToLinearHeading(new Pose2d(-37, 35, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples2 = chassi.trajectoryBuilder(ColetaSamples1.end(), true)
                .lineToLinearHeading(new Pose2d(-37, 12, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples3 = chassi.trajectoryBuilder(ColetaSamples2.end(), true)
                .lineToLinearHeading(new Pose2d(-47, 12, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples4 = chassi.trajectoryBuilder(ColetaSamples3.end(), true)
                .lineToLinearHeading(new Pose2d(-45, 55, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples5 = chassi.trajectoryBuilder(ColetaSamples4.end(), true)
                .lineToLinearHeading(new Pose2d(-45, 12, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples6 = chassi.trajectoryBuilder(ColetaSamples5.end(), true)
                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(90)))
                .build();
        Trajectory ColetaSamples7 = chassi.trajectoryBuilder(ColetaSamples6.end(), true)
                .lineToLinearHeading(new Pose2d(-55, 55, Math.toRadians(90)))
                .build();
                /*.lineToLinearHeading(new Pose2d(-62, 14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-62, 55, Math.toRadians(-90)))
                 */

        Trajectory Specimen1 = chassi.trajectoryBuilder(ColetaSamples.end(), true)
                .lineToLinearHeading(new Pose2d(-42, 55, Math.toRadians(90)))
                .build();

        Trajectory Specimen1_ = chassi.trajectoryBuilder(Specimen1.end(), true)
                .lineToLinearHeading(new Pose2d(-42, 62, Math.toRadians(90)))
                .build();

        Trajectory Specimen1T = chassi.trajectoryBuilder(Specimen1_.end(), true)
                .lineToLinearHeading(new Pose2d(0, 27, Math.toRadians(-100)))
                .build();

        Trajectory Specimen2 = chassi.trajectoryBuilder(Specimen1T.end(), true)
                .lineToLinearHeading(new Pose2d(-42, 61, Math.toRadians(90)))
                .build();

        Trajectory Specimen2T = chassi.trajectoryBuilder(Specimen2.end(), true)
                .lineToLinearHeading(new Pose2d(-4, 34, Math.toRadians(-90)))
                .build();

        Trajectory Specimen3 = chassi.trajectoryBuilder(Specimen2T.end(), true)
                .lineToLinearHeading(new Pose2d(-37, 61, Math.toRadians(90)))
                .build();

        Trajectory Specimen3T = chassi.trajectoryBuilder(Specimen3.end(), true)
                .lineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)))
                .build();

        Runnable updateLoop = () -> {
            while (opModeIsActive() && chassi.isBusy()) {
                chassi.update();
                sistemas.update();
            }
        };

        waitForStart();

        sistemas.ExpandirT();

        chassi.followTrajectoryAsync(InitialScore);
        updateLoop.run();

        sistemas.PontuarSpecimen();
        chassi.followTrajectoryAsync(ColetaSamples);
        updateLoop.run();

        chassi.followTrajectoryAsync(ColetaSamples1);
        updateLoop.run();
        chassi.followTrajectoryAsync(ColetaSamples2);
        updateLoop.run();

        chassi.followTrajectoryAsync(ColetaSamples3);
        updateLoop.run();

        chassi.followTrajectoryAsync(ColetaSamples4);
        updateLoop.run();

        chassi.followTrajectoryAsync(Specimen1);
        updateLoop.run();

        chassi.followTrajectoryAsync(Specimen1_);
        updateLoop.run();
        sistemas.ColetarSpecimen();

        sistemas.ExpandirT();
        chassi.followTrajectoryAsync(Specimen1T);
        updateLoop.run();
        sistemas.PontuarSpecimen();

        chassi.followTrajectoryAsync(Specimen2);
        updateLoop.run();
        sistemas.ColetarSpecimen();

        sistemas.ExpandirT();
        chassi.followTrajectoryAsync(Specimen2T);
        updateLoop.run();
        sistemas.PontuarSpecimen();


    }
}
