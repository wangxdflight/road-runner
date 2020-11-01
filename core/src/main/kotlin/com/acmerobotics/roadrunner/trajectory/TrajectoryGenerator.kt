package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.*
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.Log
import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Trajectory generator for creating trajectories with dynamic and static constraints from paths.
 */
object TrajectoryGenerator {

    private fun generateProfile(
        path: Path,
        constraints: TrajectoryConstraints,
        start: MotionState,
        goal: MotionState,
        resolution: Double
    ): MotionProfile {
        return MotionProfileGenerator.generateMotionProfile(start, goal, object : MotionConstraints() {
            override fun get(s: Double): SimpleMotionConstraints {
                val t = path.reparam(s)
                return constraints[
                    s,
                    path[s, t],
                    path.deriv(s, t),
                    path.secondDeriv(s, t)
                ]
            }

            override fun get(s: DoubleProgression) =
                s.zip(path.reparam(s).asIterable())
                    .map { (s, t) ->
                        constraints[
                            s,
                            path[s, t],
                            path.deriv(s, t),
                            path.secondDeriv(s, t)
                        ]
                    }
        }, resolution)
    }

    private fun generateSimpleProfile(
        constraints: DriveConstraints,
        start: MotionState,
        goal: MotionState
    ): MotionProfile {
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
            constraints.maxVel, constraints.maxAccel, constraints.maxJerk)
    }

    // note: this assumes that the profile position is monotonic increasing
    private fun displacementToTime(profile: MotionProfile, s: Double): Double {
        var tLo = 0.0
        var tHi = profile.duration()
        while (!(tLo epsilonEquals tHi)) {
            val tMid = 0.5 * (tLo + tHi)
            if (profile[tMid].x > s) {
                tHi = tMid
            } else {
                tLo = tMid
            }
        }
        return 0.5 * (tLo + tHi)
    }

    private fun pointToTime(path: Path, profile: MotionProfile, point: Vector2d) =
        displacementToTime(profile, path.project(point))

    private fun convertMarkers(
        path: Path,
        profile: MotionProfile,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): List<TrajectoryMarker> {
          Log.dbgPrint("TrajectoryGenerator, convertMarkers")
      return temporalMarkers.map { (producer, callback) ->
            TrajectoryMarker(producer.produce(profile.duration()), callback) } +
            displacementMarkers.map { (producer, callback) ->
                TrajectoryMarker(displacementToTime(profile, producer.produce(path.length())), callback) } +

            spatialMarkers.map { (point, callback) ->
                TrajectoryMarker(pointToTime(path, profile, point), callback) }
    }

    /**
     * Generate a dynamic constraint trajectory.
     * @param path path
     * @param constraints trajectory constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     * @param resolution dynamic profile sampling resolution
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateTrajectory(
        path: Path,
        constraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0),
        temporalMarkers: List<TemporalMarker> = emptyList(),
        displacementMarkers: List<DisplacementMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList(),
        resolution: Double = 0.25
    ): Trajectory {
        Log.dbgPrint("TrajectoryGenerator, generateTrajectory")

        val profile = generateProfile(path, constraints, start, goal, resolution)
        val markers = convertMarkers(path, profile, temporalMarkers, displacementMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }

    /**
     * Generate a simple constraint trajectory.
     * @param path path
     * @param constraints drive constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateSimpleTrajectory(
        path: Path,
        constraints: DriveConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0, 0.0),
        temporalMarkers: List<TemporalMarker> = emptyList(),
        displacementMarkers: List<DisplacementMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList()
    ): Trajectory {
        Log.dbgPrint("TrajectoryGenerator, generateSimpleTrajectory")

        val profile = generateSimpleProfile(constraints, start, goal)
        val markers = convertMarkers(path, profile, temporalMarkers, displacementMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }
}
