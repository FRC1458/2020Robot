package frc.team1458.robot

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.team1458.lib.util.flow.systemTimeSeconds

class RamseteFollower(
        val path: Trajectory,
        val odom: () -> Pose2d,
        b: Double,
        zeta: Double,
        goalToleranceFeet: Double = 1.0,
        goalToleranceDegrees: Double = 10.0) {

    val controller = RamseteController(b, zeta)

    val isFinished: Boolean
        get() = controller.atReference()

    init {
        controller.setTolerance(Pose2d(goalToleranceFeet, goalToleranceFeet, Rotation2d.fromDegrees(goalToleranceDegrees)))
    }

    var startTime = -100.0

    // Calculate linvel, angvel
    fun calculate(): Pair<Double, Double> {
        if (startTime < 0) {
            startTime = systemTimeSeconds
        }

        val speeds = controller.calculate(odom(), path.sample(systemTimeSeconds - startTime))

        return Pair<Double, Double>(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond)
    }
}