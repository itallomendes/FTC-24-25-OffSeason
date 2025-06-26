package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class SoColeta extends LinearOpMode {

    long temporizadorDescidaBraco = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {
        SistemasDoRobo sistemas = new SistemasDoRobo(hardwareMap, true, true);
        SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

        Pose2d posicaoInicial = new Pose2d(37.5,60, Math.toRadians(0));
        chassi.setPoseEstimate(posicaoInicial);

        sistemas.bracoMeiaAltura();
        temporizadorDescidaBraco = System.currentTimeMillis();
        while (System.currentTimeMillis() - temporizadorDescidaBraco < 2000) {
            sistemas.update();
        }

        waitForStart();

        Trajectory InitialScore = chassi.trajectoryBuilder(posicaoInicial, true)
                .lineToLinearHeading(new Pose2d(53,52,Math.toRadians(45)))
                .build();

        Trajectory Sample1 = chassi.trajectoryBuilder(InitialScore.end(), true)
                .lineToLinearHeading(new Pose2d(48,39,Math.toRadians(90)))
                .build();

        Trajectory Score1 = chassi.trajectoryBuilder(Sample1.end(), true)
                .lineToLinearHeading(new Pose2d(53,52,Math.toRadians(45)))
                .build();

        Trajectory Sample2 = chassi.trajectoryBuilder(Score1.end(), true)
                .lineToLinearHeading(new Pose2d(58,39,Math.toRadians(90)))
                .build();

        Trajectory Score2 = chassi.trajectoryBuilder(Sample2.end(), true)
                .lineToLinearHeading(new Pose2d(53,52,Math.toRadians(45)))
                .build();

        Trajectory Sample3 = chassi.trajectoryBuilder(Score2.end(), true)
                .lineToLinearHeading(new Pose2d(55,39,Math.toRadians(135)))
                .build();

        Trajectory Score3 = chassi.trajectoryBuilder(Sample3.end(), true)
                .lineToLinearHeading(new Pose2d(53,52,Math.toRadians(45)))
                .build();

        Runnable updateLoop = () -> {
            while (opModeIsActive() && chassi.isBusy()) {
                chassi.update();
                sistemas.update();
            }
        };

        sistemas.coletarSample();
        sistemas.subirBracoTransferencia();
        while (!sistemas.bracoColeta.atTarget()) {sistemas.update();}

        /* chassi.followTrajectoryAsync(Sample2);
        updateLoop.run();

        sistemas.coletarSample();

        chassi.followTrajectoryAsync(Score2);
        updateLoop.run();

        sistemas.depositar();

        chassi.followTrajectoryAsync(Sample3);
        updateLoop.run();

        sistemas.coletarSample();

        chassi.followTrajectory(Score3);
        updateLoop.run();

        sistemas.depositar();

         */
    }


}
