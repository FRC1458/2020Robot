package frc.team1458.robot


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


class ColorSensorConnect:TimedRobot() {


    private val i2cPort = I2C.Port.kOnboard
    private val m_colorSensor = ColorSensorV3(i2cPort)
    private val m_colorMatcher = ColorMatch()
    private val kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429)
    private val kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240)
    private val kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114)
    private val kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113)

    /* Robot Init
        m_colorMatcher.addColorMatch(kBlueTarget)
        m_colorMatcher.addColorMatch(kGreenTarget)
        m_colorMatcher.addColorMatch(kRedTarget)
        m_colorMatcher.addColorMatch(kYellowTarget)
    */

    /* Robot Periodic

        val current_color = m_colorSensor.getColor()

        val colorString: String
        val match = m_colorMatcher.matchClosestColor(detectedColor)
        if (match.color === kBlueTarget) {
            colorString = "Blue"
        } else if (match.color === kRedTarget) {
            colorString = "Red"
        } else if (match.color === kGreenTarget) {
            colorString = "Green"
        } else if (match.color === kYellowTarget) {
            colorString = "Yellow"
        } else {
            colorString = "Unknown"
        }

    }
    */



}
