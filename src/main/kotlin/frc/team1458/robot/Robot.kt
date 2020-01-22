package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.actuator.SRX
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.pathing.PathGenerator
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeSeconds
import frc.team1458.lib.util.maths.TurtleMaths

class Robot : TimedRobot() {

    private val oi: OI = OI()

    private val robot: RobotMap = RobotMap()

    val quantumIntake = SmartMotor.CANtalonSRX(19)

    override fun robotInit() {
        println("Robot Initialized")
        robot.drivetrain.clearOdom()
        LiveDashboard.setup(26.0, 14.0)

        SmartDashboard.putNumber("ramsete_b", robot.RAMSETE_B)
        SmartDashboard.putNumber("ramsete_zeta", robot.RAMSETE_ZETA)

        SmartDashboard.putNumber("intakevoltage", 8.0)
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
        robot.drivetrain.clearOdom()

        LiveDashboard.endPath()

        val path = robot.lowGearPathGenerator.generatePathQuintic(
                arrayOf(
                        PathGenerator.Pose(0.0, 0.0, 0.0),
                        //PathGenerator.Pose(3.5, 2.5, -60.0),
                        PathGenerator.Pose(2.0, 2.0, 0.0)
                        // PathGenerator.Pose(6.0, 2.0, 40.0)
                ),
                startVelocity = 0.0, endVelocity = 0.0, reversed = false
        )
        PathGenerator.displayOnLiveDashboard(path)

        val controller = RamseteFollower(
                path = path,
                odom = robot.drivetrain::pose,
                zeta = SmartDashboard.getNumber("ramsete_zeta", 2.80),
                b = SmartDashboard.getNumber("ramsete_b", 0.8),
                goalToleranceFeet = robot.RAMSETE_TOLERANCE_LINEAR,
                goalToleranceDegrees = robot.RAMSETE_TOLERANCE_ANGULAR)

        delay(1000)

        while(isEnabled && isAutonomous && !controller.isFinished) {
            robot.drivetrain.updateOdom()

            val (linvel, angvel) = controller.calculate()
            //break

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

        disabledInit()
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


        if(oi.xboxController.getButton(Gamepad.Button.LBUMP).triggered) {
            quantumIntake.speed = if (oi.xboxController.getButton(Gamepad.Button.A).triggered) { -0.5 } else { -0.15 }
        } else if (oi.xboxController.getButton(Gamepad.Button.RBUMP).triggered) {
            quantumIntake.speed = (0.4)
        } else {
            quantumIntake.speed = (0.0)
        }

        //quantumIntake.setVoltage(SmartDashboard.getNumber("intakevoltage", 0.0) * oi.xboxController.getButton(Gamepad.Button.A).value.toDouble())

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
        robot.drivetrain.clearOdom()

        LiveDashboard.endPath()

        val path = robot.lowGearPathGenerator.generatePathQuintic(
                arrayOf(
                        PathGenerator.Pose(0.0-7, 0.0-5, 0.0),
                        //PathGenerator.Pose(3.5-7, 2.5-5, -60.0),
                        PathGenerator.Pose(7.0-7, 5.0-5, 0.0)
                ).reversedArray(),
                startVelocity = 0.0, endVelocity = 0.0, reversed = true
        )
        PathGenerator.displayOnLiveDashboard(path)

        val controller = RamseteFollower(
                path = path,
                odom = robot.drivetrain::pose,
                zeta = robot.RAMSETE_ZETA,
                b = robot.RAMSETE_B,
                goalToleranceFeet = robot.RAMSETE_TOLERANCE_LINEAR,
                goalToleranceDegrees = robot.RAMSETE_TOLERANCE_ANGULAR)

        delay(1000)


        while(isEnabled && isTest && !controller.isFinished) {
            robot.drivetrain.updateOdom()

            val (linvel, angvel) = controller.calculate()
            //break

            robot.drivetrain.driveCmdVel(linvel, angvel)

            enabledLog()

            delay(5)
        }
        println("finished DRVIARSELO ${isEnabled} $(isTest} ${controller.isFinished}")
        robot.drivetrain.stop()
    }

    override fun testPeriodic() {


        //enabledLog()
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

        log()
    }
}

