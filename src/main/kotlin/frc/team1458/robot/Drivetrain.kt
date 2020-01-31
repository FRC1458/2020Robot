package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.team1458.lib.actuator.SRX
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.pathing.PathGenerator
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.sensor.interfaces.DistanceSensor
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.maths.TurtleMaths
import kotlin.math.IEEErem

// Notice: this code will be jank - this class should wrap as much of the jank behavior as possible,
// in order to make the interfaces for other classes easier
const val RAMSETE_BETA = 2.8
const val RAMSETE_ZETA = 0.8

const val RAMSETE_TOLERANCE_LINEAR = 0.35
const val RAMSETE_TOLERANCE_ANGULAR = 2.0


class Drivetrain(val leftMaster: SRX,
                 val rightMaster: SRX,
                 val leftMotor: SRX,
                 val rightMotor: SRX,

                 closedLoop: Boolean = false,
                 wheelDiameter: Double,
                 val trackWidth: Double,

                 var pidConstantsLowGearLeft: PIDConstants,
                 var pidConstantsLowGearRight: PIDConstants,
                 var pidConstantsHighGearLeft: PIDConstants,
                 var pidConstantsHighGearRight: PIDConstants,

                 val shifter: Solenoid? = null,
                 val gyro: AngleSensor,
                 val invertGyro: Boolean = false,

                 val maxVoltage: Double = 12.0) {

    init {
        clearOdom()

        SmartDashboard.putNumber("ramseteBeta", RAMSETE_BETA)
        SmartDashboard.putNumber("ramseteZeta", RAMSETE_ZETA)

        LiveDashboard.setup(26.0, 14.0)
    }

    val wheelCircumference = wheelDiameter.times(Math.PI)

    val leftEnc: DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = (leftMaster.encoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0)

        override val velocity: Double
            get() = leftMaster.encoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            leftMaster.encoder.zero()
        }
    }

    val rightEnc: DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = (rightMaster.encoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0)

        override val velocity: Double
            get() = rightMaster.encoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            rightMaster.encoder.zero()
        }
    }


    var highGear = false
        public get
        private set

    var closedLoop = closedLoop
        public get
        public set(value) {
            field = value;
            configPID()
        }

    val leftClosedLoopError: Double
        get() = leftMaster.closedLoopError * (wheelCircumference ?: 0.0) / 360.0

    val rightClosedLoopError: Double
        get() = rightMaster.closedLoopError * (wheelCircumference ?: 0.0) / 360.0

    init {
        highGear = false
        shifter?.retract()

        configPID()
    }

    private fun configPID() {
        val left = if (highGear) {
            pidConstantsHighGearLeft
        } else {
            pidConstantsLowGearLeft
        }
        val right = if (highGear) {
            pidConstantsHighGearRight
        } else {
            pidConstantsLowGearRight
        }

        if (closedLoop) {
            leftMaster.pidConstants = left
            rightMaster.pidConstants = right
        } else {
            leftMaster.pidConstants = left.openLoop
            rightMaster.pidConstants = right.openLoop
        }
    }

    fun shiftLowGear() {
        highGear = false
        shifter?.retract()

        configPID()
    }

    fun shiftHighGear() {
        highGear = true
        shifter?.extend()

        configPID()
    }


    @Deprecated("Use driveVoltage")
    fun driveRaw(left: Double, right: Double) {
        leftMaster.setRaw(left)
        rightMaster.setRaw(right)
    }

    // Use this when driving from a controller and your inputs are (-1.0, 1.0)
    fun driveVoltageScaled(left: Double, right: Double) {
        leftMaster.setVoltage(maxVoltage * left)
        rightMaster.setVoltage(maxVoltage * right)
    }

    fun driveVoltage(left: Double, right: Double) {
        leftMaster.setVoltage(left)
        rightMaster.setVoltage(right)
    }

    fun driveVelocity(left: Double, right: Double) {
        // Convert from ft/sec to deg/sec
        leftMaster.setVelocity(left / ((wheelCircumference ?: 0.0) / 360.0))
        rightMaster.setVelocity(right / ((wheelCircumference ?: 0.0) / 360.0))
    }

    // linvel = ft/sec, angvel = rad/sec
    fun driveCmdVel(linvel: Double, angvel: Double) {
        driveVelocity(linvel - (0.5 * trackWidth * angvel), linvel + (0.5 * trackWidth * angvel))
    }

    fun stop() {
        driveVoltage(0.0, 0.0)
    }

    /*
     * ODOMETRY/PATH FOLLOWING CODE
     */

    private val odom = DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0))

    val pose: Pose2d
        get() = odom.poseMeters
    var odomTrust: Double = 1.0 // TODO Expand (mostly here for safety and total failure mode prevention)
    // Trust: Value between [0.0, 1.0] with -1.0 representing total failure mode with no recovery

    fun clearOdom(clearGyro: Boolean = true, clearEncs: Boolean = true) {
        try {
            if (clearGyro) {
                gyro.zero()
            }

            if (clearEncs) {
                leftEnc.zero()
                rightEnc.zero()
            }

            odom.resetPosition(Pose2d(), Rotation2d.fromDegrees(0.0))
            odomTrust = 1.0
        } catch (e: Exception) {
            odomTrust = -1.0

            println("WARNING - clearOdom failed! Most likely disconnected gyro/encoders")
            // e.printStackTrace()
        } finally {}
    }

    // Update odometry - call this at the end of teleopPeriodic
    fun updateOdom(dashboard: Boolean = true) {
        fun getHeading() = gyro.angle.IEEErem(360.0) * (if (invertGyro) {
            -1.0
        } else {
            1.0
        })

        fun getHeadingRate() = gyro.rate * (if (invertGyro) {
            -1.0
        } else {
            1.0
        })

        try {
            odom.update(Rotation2d.fromDegrees(getHeading()), leftEnc.distanceFeet, rightEnc.distanceFeet)

            // TODO although this says "poseMeters", we are actually using feet for our measurements
            if (dashboard) {
                LiveDashboard.putOdom(pose.translation.x, pose.translation.y, pose.rotation.radians)
                LiveDashboard.putPath(pose.translation.x, pose.translation.y, pose.rotation.radians)
            }

            if (odomTrust != -1.0) {
                odomTrust = TurtleMaths.constrain(max = 1.0, min = 0.0, value = odomTrust + 0.05)
            }
        } catch (e: Exception) {
            if (odomTrust != -1.0) {
                odomTrust = TurtleMaths.constrain(max = 1.0, min = 0.0, value = odomTrust - 0.20)
            }

            println("WARNING - updateOdom failed! Most likely disconnected gyro/encoders")
            // e.printStackTrace()
        } finally { }
    }

    // TODO Add time limit to this function so we dont auto forever
    fun followRamsteBlocking(robot: Robot, path: Trajectory, loopEvalLambda: () -> Boolean = { true },
                             logAndDashboard: Boolean = true, clearOdom: Boolean = true, delayMs: Double = 5.0,
                             odomTrustThreshold: Double = 0.20) {
        if (clearOdom) {
            clearOdom()
        }

        if (logAndDashboard) {
            LiveDashboard.endPath()
            PathGenerator.displayOnLiveDashboard(path)
        }

        // TODO Probably remove/hardcode these
        val z = SmartDashboard.getNumber("ramseteZeta", RAMSETE_ZETA)
        val b = SmartDashboard.getNumber("ramseteBeta", RAMSETE_BETA)

        val controller = RamseteFollower(
                path = path,
                odom = { pose },
                zeta = z,
                b = b,
                goalToleranceFeet = RAMSETE_TOLERANCE_LINEAR,
                goalToleranceDegrees = RAMSETE_TOLERANCE_ANGULAR)

        while (robot.isEnabled && !controller.isFinished && loopEvalLambda() && odomTrust >= odomTrustThreshold) {
            updateOdom()
            val (linVel, angVel) = controller.calculate()

            driveCmdVel(linVel, angVel)

            if (logAndDashboard) {
                robot.enabledLog()
            }

            if (odomTrust != 1.0) {
                println("WARNING - SOME PATH FOLLOWING ODOMETRY UPDATES FAILING ($odomTrust)")
            }

            delay(delayMs)
        }

        stop()

        if (odomTrust < odomTrustThreshold) {
            println("ERROR - PATH FOLLOWING ABORTED: TRUST THRESHOLD EXCEEDED ($odomTrust)")
        }
    }


    // TODO drive for vision

    // TODO turn in place

    // TODO drive const distance
}