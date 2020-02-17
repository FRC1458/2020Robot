package frc.team1458.robot

//Recommended imports
import edu.wpi.first.wpilibj.I2C
import com.revrobotics.ColorSensorV3
import com.revrobotics.ColorMatch



/*

Available functions:

.update() -> Returns current color detected as a string

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

        if (match.color === kBlueTarget) return("Blue")
        if (match.color === kRedTarget) return("Red")
        if (match.color === kGreenTarget) return("Green")
        if (match.color === kYellowTarget) return("Yellow")
        return("turtwig")
    }
}
