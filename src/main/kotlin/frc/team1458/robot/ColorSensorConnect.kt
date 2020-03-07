package frc.team1458.robot

//Recommended imports
import edu.wpi.first.wpilibj.I2C
import com.revrobotics.ColorSensorV3
import com.revrobotics.ColorMatch

private val robot: RobotMap = RobotMap()


/*

Available functions:

.update() -> Returns current color detected as a string
.rotateMotor(colorDesired, currentColor, speed, override) -> Rotate the wheel until color desired is reached.
 - Speed sets motor speed
 - Override allows speed >1

*/

class ColorSensorConnect {

    //Establish Variables and color targets
    val i2cPort = I2C.Port.kOnboard
    val m_colorSensor = ColorSensorV3(i2cPort)
    val m_colorMatcher = ColorMatch()
    val kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429)
    val kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240)
    val kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114)
    val kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113)

    //Init simply adds the color values to the matcher as part of the documentation
    init {
        m_colorMatcher.addColorMatch(kBlueTarget)
        m_colorMatcher.addColorMatch(kGreenTarget)
        m_colorMatcher.addColorMatch(kRedTarget)
        m_colorMatcher.addColorMatch(kYellowTarget)
    }


    fun update(): String {
        //Match the color and compare it to our predetermined RGB values
        val match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor())

        if (match.color === kBlueTarget) return ("Blue")
        if (match.color === kRedTarget) return ("Red")
        if (match.color === kGreenTarget) return ("Green")
        if (match.color === kYellowTarget) return ("Yellow")

        //Error handling
        return ("turtwig: NO COLOR")
    }

    fun rotateMotor(colorDesired: String, currentColor: String, speed: Double, override: Boolean = false) {

        var colorNow: String = currentColor

        while (colorDesired != colorNow && (speed < 1 || override)) {
            robot.colorMotor.setRaw(speed)
            colorNow = update()
        }

    }
}
