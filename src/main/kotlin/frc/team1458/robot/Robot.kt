package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.pathing.PathGenerator
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeSeconds
import frc.team1458.lib.util.maths.TurtleMaths

class Robot : TimedRobot() {

    private val oi: OI = OI()

    private val robot: RobotMap = RobotMap()

    override fun robotInit() {
        println("Robot Initialized")
        robot.drivetrain.clearOdom()
        LiveDashboard.setup(3.0, 14.0)
    }

    fun log() {

        SmartDashboard.putNumber("Left Distance", robot.drivetrain.leftEnc.distanceFeet)
        SmartDashboard.putNumber("Right Distance", robot.drivetrain.rightEnc.distanceFeet)

        SmartDashboard.putNumber("Left Velocity", robot.drivetrain.leftEnc.velocityFeetSec)
        SmartDashboard.putNumber("Right Velocity", robot.drivetrain.rightEnc.velocityFeetSec)

        SmartDashboard.putNumber("Left Error", robot.drivetrain.leftClosedLoopError)
        SmartDashboard.putNumber("Right Error", robot.drivetrain.rightClosedLoopError)

        SmartDashboard.putNumber("Odom X Feet", robot.drivetrain.pose.translation.x)
        SmartDashboard.putNumber("Odom Y Feet", robot.drivetrain.pose.translation.y)
        SmartDashboard.putNumber("Odom Theta Deg", robot.drivetrain.pose.rotation.degrees)

        SmartDashboard.putNumber("NavX Theta", robot.drivetrain.gyro.angle)
    }

    fun enabledLog() {

        log()
    }

    override fun autonomousInit() {

        val path = robot.lowGearPathGenerator.generatePathQuintic(
                arrayOf(
                        PathGenerator.Pose(0.0, 0.0, 0.0),
                        PathGenerator.Pose(2.0, 2.0, 0.0)
                ),
                startVelocity = 0.0, endVelocity = 0.0, reversed = false
        )
        PathGenerator.displayOnLiveDashboard(path)

        val controller = RamseteFollower(
                path = path,
                odom = robot.drivetrain::pose,
                zeta = robot.RAMSETE_ZETA,
                b = robot.RAMSETE_B,
                goalToleranceFeet = robot.RAMSETE_TOLERANCE_LINEAR,
                goalToleranceDegrees = robot.RAMSETE_TOLERANCE_ANGULAR)

        while(isEnabled && isAutonomous && !controller.isFinished && false) { // TODO remove false so this thing runs
            robot.drivetrain.updateOdom()

            val (linvel, angvel) = controller.calculate()
            robot.drivetrain.driveCmdVel(linvel, angvel)

            enabledLog()

            delay(5)
        }
        robot.drivetrain.stop()
    }

    override fun autonomousPeriodic() {

        /*
        val start = systemTimeSeconds
        while((systemTimeSeconds - start) < 6.0) {
            val v = 3.0 - Math.abs((systemTimeSeconds - start) - 3)
            robot.drivetrain.driveVelocity(v, v)

            enabledLog()
            SmartDashboard.putNumber("Time", (systemTimeSeconds - start));
            SmartDashboard.putNumber("Setpoint", v);
        }
        */


    }

    override fun teleopInit() {
        // do nothing maladors
    }

    // TODO make this faster than 20ms
    override fun teleopPeriodic() {
        val (left, right) = TurtleMaths.arcadeDrive(TurtleMaths.deadband(oi.throttle.value, 0.05), TurtleMaths.deadband(oi.steer.value, 0.05))
        //robot.drivetrain.driveVoltageScaled(left, right)
        robot.drivetrain.driveVelocity(6.0 * left, 6.0 * right)

        robot.drivetrain.updateOdom()

        // TODO get out of this and put in log function
        SmartDashboard.putNumber("Speed", oi.throttle.value)
        SmartDashboard.putNumber("Steer", oi.steer.value)

        SmartDashboard.putNumber("Left Arcade", left * 6.0)
        SmartDashboard.putNumber("Right Arcade", right * 6.0)

        SmartDashboard.putNumber("Left Buffer", robot.drivetrain.leftMaster.inst.closedLoopTarget)
        SmartDashboard.putNumber("Right Buffer", robot.drivetrain.rightMaster.inst.closedLoopTarget)

        enabledLog()
    }

    override fun testInit() {

    }

    override fun testPeriodic() {


        enabledLog()
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

        log()
    }
}

