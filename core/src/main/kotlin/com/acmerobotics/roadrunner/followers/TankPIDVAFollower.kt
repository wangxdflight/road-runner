package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.util.Log
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.sign

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, one feedback loop controls the path displacement (that is, x in the robot reference frame), and
 * another feedback loop to minimize cross track (lateral) error via heading correction (overall, very similar to
 * [HolonomicPIDVAFollower] except adjusted for the nonholonomic constraint). Feedforward is applied at the wheel level.
 *
 * @param axialCoeffs PID coefficients for the robot axial (robot X) controller
 * @param crossTrackCoeffs PID coefficients for the robot heading controller based on cross track error
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
class TankPIDVAFollower @JvmOverloads constructor(
    axialCoeffs: PIDCoefficients,
    crossTrackCoeffs: PIDCoefficients,
    admissibleError: Pose2d = Pose2d(),
    timeout: Double = 0.0,
    clock: NanoClock = NanoClock.system()
) : TrajectoryFollower(admissibleError, timeout, clock) {
    private val axialController = PIDFController(axialCoeffs)
    private val crossTrackController = PIDFController(crossTrackCoeffs)
    private val headingController = PIDFController(crossTrackCoeffs)

    override var lastError: Pose2d = Pose2d()
    init {
        headingController.setInputBounds(-Math.PI, Math.PI)
    }
    override fun followTrajectory(trajectory: Trajectory) {
        Log.dbgPrint("TankPIDVAFollower: followTrajectory, to reset PIDController first and call parent follower")

        axialController.reset()
        crossTrackController.reset()
        headingController.reset()

        super.followTrajectory(trajectory)
    }

    override fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal {
        Log.dbgPrint("TankPIDVAFollower: internalUpdate");
        Log.dbgPrint("  to get target vel, accel (fieldToRobotVelocity) from trajectory, and then targetRobotVel/Accel")
        Log.dbgPrint("  and then calculatePoseError")
        Log.dbgPrint("  and then position PID controller update, finally drive signal")
        val t = elapsedTime()

        val targetPose = trajectory[t]
        val targetVel = trajectory.velocity(t)
        val targetAccel = trajectory.acceleration(t)
        Log.dbgPrint("targetPose: ".plus(targetPose.toString()))
        Log.dbgPrint("targetVel: ".plus(targetVel.toString()))
        Log.dbgPrint("targetAccel: ".plus(targetAccel.toString()))

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        val poseError = Kinematics.calculateRobotPoseError(targetPose, currentPose)

        // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
        axialController.targetPosition = poseError.x
        crossTrackController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        axialController.targetVelocity = targetRobotVel.x
        crossTrackController.targetVelocity = targetRobotVel.y
        headingController.targetVelocity = targetRobotVel.heading

        // note: feedforward is processed at the wheel level
        val axialCorrection = axialController.update(0.0, currentRobotVel?.x)
        //val headingCorrection = sign(targetVel.vec() dot currentPose.vec()) *
        //    crossTrackController.update(0.0, currentRobotVel?.y)
        val crossTrackCorrection = crossTrackController.update(0.0, currentRobotVel?.y)
        val headingCorrection = headingController.update(0.0, currentRobotVel?.heading)

        val correctedVelocity = targetRobotVel + Pose2d(
            axialCorrection,
                crossTrackCorrection,
            headingCorrection
        )

        lastError = poseError
        Log.dbgPrint("lastPoseError: ".plus(lastError.toString()))
        Log.dbgPrint("axialCorrection: ".plus(axialCorrection.toString()))
        Log.dbgPrint("crossTrackCorrection: ".plus(crossTrackCorrection.toString()))
        Log.dbgPrint("headingCorrection: ".plus(headingCorrection.toString()))
        return DriveSignal(correctedVelocity, targetRobotAccel)
    }
}
