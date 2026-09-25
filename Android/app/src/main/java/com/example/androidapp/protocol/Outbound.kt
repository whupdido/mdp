package com.example.androidapp.protocol

import com.example.androidapp.arena.Facing
import com.example.androidapp.arena.Task

/**
 * Every string the app sends lives here, so when the team settles the final
 * command vocabulary there is exactly one file to change.
 *
 * Map messages follow the ARCM briefing slides, because that is what the
 * supervisor will have in mind at the AMD tool. Motion commands follow
 * STM32_motion_spec.md on the main branch.
 */
object Outbound {

    // --- map, C.6 and C.7 ------------------------------------------------

    /** Obstacle placed or moved, sent once on finger lift. */
    fun add(id: Int, x: Int, y: Int) = "ADD,B$id,($x,$y)"

    /** Obstacle dragged off the arena. */
    fun sub(id: Int) = "SUB,B$id"

    /** Target face annotated. */
    fun face(id: Int, facing: Facing) = "FACE,B$id,${facing.letter}"

    // --- motion, C.3 -----------------------------------------------------
    //
    // The STM board takes a two-letter verb and three digits, one command at a
    // time, and replies DONE when the move finishes. It cannot turn on the
    // spot: a 90-degree turn also carries the car 2.5-3.2 cells forward or
    // back, which is why the turn buttons are labelled "forward-left" rather
    // than "left".

    fun forward(cm: Int) = "FW${cm.pad3()}"
    fun backward(cm: Int) = "BW${cm.pad3()}"
    fun forwardLeft(deg: Int) = "FL${deg.pad3()}"
    fun forwardRight(deg: Int) = "FR${deg.pad3()}"
    fun backLeft(deg: Int) = "BL${deg.pad3()}"
    fun backRight(deg: Int) = "BR${deg.pad3()}"

    const val STOP = "STOP"

    // --- Task 1 ----------------------------------------------------------

    /**
     * Begin the Task 1 attempt: plan a route from the obstacles already sent,
     * then drive it.
     *
     * The rules require this to come from a button on the tablet -- during the
     * run the team may not touch anything else, so a keypress on the laptop's
     * SSH session is not allowed to be what starts the robot.
     * `rpi/run_task1.py` waits for this string.
     */
    const val START = "START"

    /**
     * Begin the Task 2 attempt: out of the carpark, round both obstacles
     * according to the arrows, and back into the carpark, inside three
     * minutes.
     *
     * Same rule as [START] -- the run may only be triggered from the tablet.
     * Task 2 keys in nothing beforehand: the obstacles are placed after the
     * prep time and their distances are withheld, so this carries no payload.
     */
    const val START2 = "START2"

    fun start(task: Task) = if (task == Task.TASK2) START2 else START

    private fun Int.pad3(): String = coerceIn(0, 999).toString().padStart(3, '0')
}

/** The moves the D-pad can issue. */
enum class Move { FORWARD, BACKWARD, FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT, STOP }

fun Move.toCommand(distanceCm: Int, angleDeg: Int): String = when (this) {
    Move.FORWARD -> Outbound.forward(distanceCm)
    Move.BACKWARD -> Outbound.backward(distanceCm)
    Move.FWD_LEFT -> Outbound.forwardLeft(angleDeg)
    Move.FWD_RIGHT -> Outbound.forwardRight(angleDeg)
    Move.BACK_LEFT -> Outbound.backLeft(angleDeg)
    Move.BACK_RIGHT -> Outbound.backRight(angleDeg)
    Move.STOP -> Outbound.STOP
}
