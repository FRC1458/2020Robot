package frc.team1458.lib.pathing

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.go
import frc.team1458.lib.util.flow.systemTimeSeconds
import java.io.BufferedWriter
import java.io.FileWriter
import java.lang.Double.max

// Should be represented in units of Volts, Feet, Seconds. NOT TALON SRX UNITS
class PathGenerator(
        // Characterization constants
        val left_kS: Double,
        val right_kS: Double,
        val left_kV: Double,
        val right_kV: Double,
        val left_kA: Double,
        val right_kA: Double,

        val trackWidth: Double,

        // Try to keep this a few volts lower than nominal battery voltage
        val maxControlEffortVolts: Double = 10.0,
        val maxVelocity: Double, // ft/sec
        val maxAcceleration: Double, // ft/sec^2
        val maxCentripitalAcceleration: Double // ft/sec^2 - prevents slippy slippy
) {

    val driveKinematics = DifferentialDriveKinematics(trackWidth)

    val voltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforward(max(left_kS, right_kS), max(left_kV, right_kV), max(left_kA, right_kA)), // Most conservative config
            driveKinematics,
            maxControlEffortVolts
    )

    val centripetalAccelerationConstraint = CentripetalAccelerationConstraint(maxCentripitalAcceleration)

    val trajectoryConfig = TrajectoryConfig(maxVelocity, maxAcceleration)
            .setKinematics(driveKinematics)
            .addConstraint(voltageConstraint)
            .addConstraint(centripetalAccelerationConstraint)


    // TODO UNTESTED, MAY BE BLOCKING

    // Start and end point have orientation and position, rest just have position
    fun generatePathClampedCubic(initial: Pose, waypoints: Array<Position>, final: Pose, startVelocity: Double = 0.0, endVelocity: Double = 0.0, reversed: Boolean = false): Trajectory {
        trajectoryConfig.setStartVelocity(startVelocity)
        trajectoryConfig.setEndVelocity(endVelocity)
        trajectoryConfig.setReversed(reversed)

        return TrajectoryGenerator.generateTrajectory(initial.asPose2d, waypoints.map { it.asTranslation2d }, final.asPose2d, trajectoryConfig)
    }

    // All points have position and orientation
    fun generatePathQuintic(waypoints: Array<Pose>, startVelocity: Double = 0.0, endVelocity: Double = 0.0, reversed: Boolean = false): Trajectory {
        trajectoryConfig.setStartVelocity(startVelocity)
        trajectoryConfig.setEndVelocity(endVelocity)
        trajectoryConfig.setReversed(reversed)

        return TrajectoryGenerator.generateTrajectory(waypoints.map { it.asPose2d }, trajectoryConfig)
    }

    class Position(val x: Double, val y: Double) {
        val asTranslation2d: Translation2d
            get() = Translation2d(x, y)
    }

    class Pose(val x: Double, val y: Double, val thetaDegrees: Double) {
        val asPose2d: Pose2d
            get() = Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees))
    }

    companion object {
        // Writes path to file. Uses a coroutine
        // TODO UNTESTED
        fun writeToFile(filePath: String, path: Trajectory) {
            go {
                try {
                    val logFile = FileWriter(filePath)
                    val logWriter = BufferedWriter(logFile)

                    logWriter.appendln("time,x,y,theta,velocity,angular_velocity,acceleration");

                    var t: Double = 0.0
                    while (t < path.totalTimeSeconds) {
                        val s = path.sample(t)
                        logWriter.appendln("${s.timeSeconds},${s.poseMeters.translation.x},${s.poseMeters.translation.y},${s.velocityMetersPerSecond},${s.curvatureRadPerMeter * s.velocityMetersPerSecond},${s.accelerationMetersPerSecondSq}")
                        t += 0.05
                    }

                    logWriter.flush()
                    logWriter.close()
                    logFile.close()

                } catch (e: Exception) {
                    println("Error in write to file")
                    e.printStackTrace()
                }
            }
        }


        // Displays on dashboard at 1x speed
        // THIS IS BLOCKING
        // TODO untested
        fun displayOnLiveDashboard(path: Trajectory) {
            val startTime = systemTimeSeconds
            var time = systemTimeSeconds - startTime

            while(time < path.totalTimeSeconds) {
                val pose = path.sample(time).poseMeters
                LiveDashboard.putOdom(pose.translation.x, pose.translation.y, pose.rotation.radians)
                LiveDashboard.putPath(pose.translation.x, pose.translation.y, pose.rotation.radians)
                delay(15)

                time = systemTimeSeconds - startTime
            }
        }
    }
}