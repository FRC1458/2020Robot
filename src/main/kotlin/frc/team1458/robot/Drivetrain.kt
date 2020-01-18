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
import frc.team1458.lib.actuator.SRX
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.sensor.interfaces.DistanceSensor
import frc.team1458.lib.util.LiveDashboard

// Notice: this code will be jank - this class should wrap as much of the jank behavior as possible,
// in order to make the interfaces for other classes easier

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

                 val shifter : Solenoid? = null,
                 val gyro : AngleSensor,
                 val invertGyro : Boolean = false,

                 val maxVoltage : Double = 12.0) {

    val wheelCircumference = wheelDiameter.times(Math.PI)

    val leftEnc : DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = (leftMaster.encoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0)

        override val velocity: Double
            get() = leftMaster.encoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            leftMaster.encoder.zero()
        }
    }

    val rightEnc : DistanceSensor = object : DistanceSensor {
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
        public get() = leftMaster.closedLoopError * (wheelCircumference ?: 0.0) / 360.0

    val rightClosedLoopError: Double
        public get() = rightMaster.closedLoopError * (wheelCircumference ?: 0.0) / 360.0

    init {
        highGear = false
        shifter?.retract()

        configPID()
    }

    fun configPID() {
        val left = if(highGear) { pidConstantsHighGearLeft } else { pidConstantsLowGearLeft }
        val right = if(highGear) { pidConstantsHighGearRight } else { pidConstantsLowGearRight }

        if(closedLoop) {
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
        driveVelocity(linvel + (0.5 * trackWidth * angvel), linvel - (0.5 * trackWidth * angvel))
    }

    fun stop() {
        driveRaw(0.0, 0.0)
    }


    val odom = DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0));

    val pose: Pose2d
        public get() = odom.poseMeters

    fun clearOdom(clearGyro: Boolean = true, clearEncs: Boolean = true) {
        if (clearGyro) {
            gyro.zero()
        }
        if (clearEncs) {
            leftEnc.zero()
            rightEnc.zero()
        }
    }


    // Update odometry - call this at the end of teleopPeriodic
    fun updateOdom() {
        fun getHeading() = Math.IEEEremainder(gyro.angle, 360.0) * (if(invertGyro) { -1.0 } else { 1.0 })
        fun getHeadingRate() = gyro.rate * (if(invertGyro) { -1.0 } else { 1.0 })

        odom.update(Rotation2d.fromDegrees(getHeading()), leftEnc.distanceFeet, rightEnc.distanceFeet)

        // although this says "poseMeters", we are actually using feet for our measurements
        LiveDashboard.putOdom(pose.translation.x, pose.translation.y, pose.rotation.radians)
        LiveDashboard.putPath(pose.translation.x, pose.translation.y, pose.rotation.radians)
    }

    fun resetOdom() {
        odom.resetPosition(Pose2d(), Rotation2d.fromDegrees(0.0))
    }

    // TODO drive for vision

    // TODO turn in place

    // TODO drive const distance

    // TODO ramsete + odom
}