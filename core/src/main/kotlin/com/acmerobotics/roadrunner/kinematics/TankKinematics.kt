package com.acmerobotics.roadrunner.kinematics

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Log
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sin
import kotlin.math.cos

/**
 * Tank drive kinematic equations based upon the unicycle model. All wheel positions and velocities are given in
 * (left, right) tuples. Robot poses are specified in a coordinate system with positive x pointing forward, positive y
 * pointing left, and positive heading measured counter-clockwise from the x-axis.
 *
 * [This page](http://rossum.sourceforge.net/papers/DiffSteer/) gives a motivated derivation.
 */
object TankKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotVel] given [trackWidth].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */

    @JvmStatic
    fun robotToWheelVelocities(robotVel: Pose2d, trackWidth: Double): List<Double> {
        Log.dbgPrint(3);
        Log.dbgPrint("robotToWheelVelocities: robotVel or robotAccel ".plus(robotVel.toString()))
        var R: Double = 0.0
        var L: Double  = 0.0
        //require((robotVel.y epsilonEquals 0.0)) { "Lateral (robot y) velocity must be zero for tank drives" }
        val theta = robotVel.heading
        if (theta epsilonEquals 0.0) {
            var t = robotVel.x
            if (abs(robotVel.x) < abs(robotVel.y))
                t = robotVel.y
            R = t
            L = R
            Log.dbgPrint("robotToWheelVelocities going stright???")
        }
        else {
            val deltaX = robotVel.x
            val deltaY = robotVel.y
            var rt = deltaX / sin(theta)  // swap X, Y
            if (abs(robotVel.x) < abs(robotVel.y)) {
                Log.dbgPrint("robotToWheelVelocities robotVel.y > robotVel.x")
                //rt = deltaY / (cos(theta) - 1)
            }

            //Log.dbgPrint("robotToWheelVelocities: rt ".plus(rt.toString()).plus(" "))
            val r_minus_l = trackWidth * theta
            val r_plus_l = rt / (trackWidth / 2) * r_minus_l
            R = (r_plus_l + r_minus_l) / 2
            L = (r_plus_l - r_minus_l) / 2

        }
        val leftWheel = robotVel.x - trackWidth / 2 * robotVel.heading
        val rightWheel = robotVel.x + trackWidth / 2 * robotVel.heading

        Log.dbgPrint("robotToWheelVelocities: (original) ".plus(leftWheel.toString())
                .plus(" ").plus(rightWheel.toString()))
        Log.dbgPrint("robotToWheelVelocities: (new) ".plus(L.toString())
                .plus(" ").plus(R.toString()))
        //return listOf(leftWheel, rightWheel)
        return listOf(L, R)
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given [trackWidth].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */
    // follows from linearity of the derivative
    @JvmStatic
    fun robotToWheelAccelerations(robotAccel: Pose2d, trackWidth: Double) : List<Double>
    {
        Log.dbgPrint("robotToWheelAccelerations");
        return robotToWheelVelocities(robotAccel, trackWidth)
    }

    /**
     * Computes the robot velocity corresponding to [wheelVelocities] and the given drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */
    @JvmStatic
    fun wheelToRobotVelocities(wheelVelocities: List<Double>, trackWidth: Double): Pose2d {
        val (left, right) = wheelVelocities
        return Pose2d(
            (left + right) / 2.0,
            0.0,
            (-left + right) / trackWidth
        )
    }
}
