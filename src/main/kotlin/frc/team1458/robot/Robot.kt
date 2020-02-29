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
        SmartDashboard.putNumber("intakevoltage", 0.8)

    }

    fun log() {

        // TODO this makes me sad
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
        // TODO so does this
        log()
    }

    override fun autonomousInit() {
        val path = robot.lowGearPathGenerator.generatePathQuintic(
                arrayOf(
                        PathGenerator.Pose(0.0, 0.0, 0.0),
                        //PathGenerator.Pose(3.5, 2.5, -60.0),
                        PathGenerator.Pose(2.0, 2.0, 0.0)
                        // PathGenerator.Pose(6.0, 2.0, 40.0)
                ),
                startVelocity = 0.0, endVelocity = 0.0, reversed = false
        )

        // TODO Test
        robot.drivetrain.followRamseteBlocking(this, path)
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

        when {
            oi.xboxController.getButton(Gamepad.Button.LBUMP).triggered -> {
                quantumIntake.speed = SmartDashboard.getNumber("intakevoltage", 0.0) // if (oi.xboxController.getButton(Gamepad.Button.A).triggered) { intakevoltage } else { -0.15 }
            }
            oi.xboxController.getButton(Gamepad.Button.RBUMP).triggered -> {
                quantumIntake.speed = -SmartDashboard.getNumber("intakevoltage", 0.0)
            }
            else -> {
                quantumIntake.speed = (0.0)
            }
        }

        //quantumIntake.setVoltage(SmartDashboard.getNumber("intakevoltage", 0.0) * oi.xboxController.getButton(Gamepad.Button.A).value.toDouble())




        //Color sensing.


        //Gets the current color and also updates SmartDashboard with the string value
        val color = robot.colorSense.update()

        //TODO Remove the print. It is for testing only
        println(color)
        SmartDashboard.putString("Color Detected", color)

        //Imagine making a motor do things
        //This is mega stonks

        robot.colorSense.rotateMotor("Blue", color, .1)






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
        //robot.drivetrain.turnInPlaceToAngle(this,180.0,1.0,5.0)
        robot.drivetrain.leftMotor.setRaw(0.3)
        delay(1500)
        robot.drivetrain.leftMotor.setRaw(0.0)

        delay(1500)

        robot.drivetrain.rightMotor.setRaw(0.3)
        delay(1500)
        robot.drivetrain.rightMotor.setRaw(0.0)
    }

    override fun testPeriodic() {
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

        log()
    }
}

