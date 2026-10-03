package com.example.androidapp.arena

/**
 * Pure-Kotlin arena model. Deliberately free of Android imports so it can be
 * unit-tested on the JVM in about a second instead of via a 90-second install.
 *
 * Coordinate convention (agreed with the team, see Android/PROTOCOL.md):
 *   - 20 x 20 cells, each cell 10 cm, arena 2.0 m x 2.0 m
 *   - (0,0) is the BOTTOM-LEFT cell; x grows East, y grows North
 *   - an obstacle is exactly one cell, and its (x,y) is that cell
 *   - the robot is 3 x 3 cells, and its (x,y) is the CENTRE cell
 *   - the start zone is x,y in 0..2; the robot starts at (1,1) facing N
 */

/** [bearingDeg] is a compass bearing: 0 is North, increasing clockwise. */
enum class Facing(val letter: Char, val bearingDeg: Float) {
    N('N', 0f), E('E', 90f), S('S', 180f), W('W', 270f);

    fun turnedLeft(): Facing = entries[(ordinal + 3) % 4]
    fun turnedRight(): Facing = entries[(ordinal + 1) % 4]

    companion object {
        fun from(token: String): Facing? = when (token.trim().uppercase()) {
            "N", "NORTH", "UP" -> N
            "E", "EAST", "RIGHT" -> E
            "S", "SOUTH", "DOWN" -> S
            "W", "WEST", "LEFT" -> W
            else -> null
        }
    }
}

data class Obstacle(
    val id: Int,
    val x: Int,
    val y: Int,
    /** Which face the team believes holds the target image (C.7). */
    val targetFace: Facing? = null,
    /** Target ID 11..40, once the robot reports it (C.9). */
    val targetId: Int? = null,
)

data class RobotPose(val x: Int, val y: Int, val facing: Facing)

/**
 * One recorded moment of a run, for the replay scrubber.
 *
 * Kept as whole snapshots rather than deltas: a run is a few hundred frames at
 * most, and being able to jump straight to any index without replaying the
 * history to get there is worth far more than the memory.
 */
data class RunFrame(
    val atMs: Long,
    val robot: RobotPose,
    val trail: List<Pair<Int, Int>>,
    val obstacles: List<Obstacle>,
    /** What happened at this moment, if anything worth captioning. */
    val note: String? = null,
)

data class ArenaState(
    val obstacles: List<Obstacle> = emptyList(),
    val robot: RobotPose = Arena.START_POSE,
    /** Previous robot cells, oldest first — drawn as a faint trail. */
    val trail: List<Pair<Int, Int>> = emptyList(),
) {
    fun obstacleAt(x: Int, y: Int): Obstacle? = obstacles.firstOrNull { it.x == x && it.y == y }
    fun obstacle(id: Int): Obstacle? = obstacles.firstOrNull { it.id == id }
}

object Arena {

    /** Cells per side. */
    const val SIZE = 20

    /** Physical size of one cell, in centimetres. */
    const val CELL_CM = 10

    /** The robot footprint is 3 x 3 cells (the car is 23.0 x 18.8 cm). */
    const val ROBOT_SPAN = 3

    /**
     * The start zone is the 4 x 4 block at the origin corner — 40 cm.
     *
     * Matched to algorithm/constants.py START_ZONE_SIZE_CM, which is the
     * functional definition: the planner refuses to place obstacles inside it.
     * This end only shades the area, so the planner's number wins. Worth one
     * person confirming against the physical arena.
     */
    const val START_ZONE_SPAN = 4

    val START_POSE = RobotPose(1, 1, Facing.N)

    /** Valid target IDs from the image pool: 11-19 digits, 20-35 letters, 36-40 arrows/stop. */
    val TARGET_ID_RANGE = 11..40

    /**
     * The character actually printed on each target image, from the pool table
     * in the MDP briefing.
     *
     * Showing "25" tells you an ID. Showing "F" as well tells you what the
     * robot's camera was looking at, which is the thing a human in the lab
     * actually wants to check against the physical block.
     */
    private val GLYPHS: Map<Int, String> = buildMap {
        (11..19).forEach { put(it, (it - 10).toString()) }          // 1 - 9
        listOf("A", "B", "C", "D", "E", "F", "G", "H")              // 20 - 27
            .forEachIndexed { i, g -> put(20 + i, g) }
        listOf("S", "T", "U", "V", "W", "X", "Y", "Z")              // 28 - 35
            .forEachIndexed { i, g -> put(28 + i, g) }
        put(36, "↑")                                            // up
        put(37, "↓")                                            // down
        put(38, "→")                                            // right
        put(39, "←")                                            // left
        put(40, "●")                                            // stop
    }

    /** The printed character for a target ID, or null if the ID is not in the pool. */
    fun glyphFor(targetId: Int): String? = GLYPHS[targetId]

    fun inBounds(x: Int, y: Int): Boolean = x in 0 until SIZE && y in 0 until SIZE

    /**
     * A 3x3 robot centred on (x,y) must keep every cell inside the arena,
     * so legal centres run 1..18 rather than 0..19.
     */
    fun isLegalRobotCentre(x: Int, y: Int): Boolean {
        val margin = ROBOT_SPAN / 2
        return x in margin until (SIZE - margin) && y in margin until (SIZE - margin)
    }

    fun isStartZone(x: Int, y: Int): Boolean = x < START_ZONE_SPAN && y < START_ZONE_SPAN

    /**
     * Lowest unused obstacle number. Numbers are never recycled while an obstacle
     * is alive and survivors are never renumbered — the robot has already been
     * told about them by number.
     */
    fun nextFreeId(obstacles: List<Obstacle>): Int {
        val taken = obstacles.mapTo(HashSet()) { it.id }
        var candidate = 1
        while (candidate in taken) candidate++
        return candidate
    }
}

// ---------------------------------------------------------------------------
// Transitions. Each returns a new state; none of them mutate.
// ---------------------------------------------------------------------------

fun ArenaState.withObstacleAdded(x: Int, y: Int): Pair<ArenaState, Obstacle>? {
    if (!Arena.inBounds(x, y)) return null
    if (obstacleAt(x, y) != null) return null
    val created = Obstacle(id = Arena.nextFreeId(obstacles), x = x, y = y)
    return copy(obstacles = obstacles + created) to created
}

fun ArenaState.withObstacleMoved(id: Int, x: Int, y: Int): ArenaState? {
    if (!Arena.inBounds(x, y)) return null
    val existing = obstacle(id) ?: return null
    val blocker = obstacleAt(x, y)
    if (blocker != null && blocker.id != id) return null
    if (existing.x == x && existing.y == y) return this
    return copy(obstacles = obstacles.map { if (it.id == id) it.copy(x = x, y = y) else it })
}

fun ArenaState.withObstacleRemoved(id: Int): ArenaState =
    copy(obstacles = obstacles.filterNot { it.id == id })

fun ArenaState.withTargetFace(id: Int, face: Facing?): ArenaState =
    copy(obstacles = obstacles.map { if (it.id == id) it.copy(targetFace = face) else it })

/** Applies an inbound TARGET report (C.9). A face of null leaves any existing face alone. */
fun ArenaState.withTargetReported(id: Int, targetId: Int, face: Facing?): ArenaState? {
    if (targetId !in Arena.TARGET_ID_RANGE) return null
    if (obstacle(id) == null) return null
    return copy(
        obstacles = obstacles.map {
            if (it.id == id) it.copy(targetId = targetId, targetFace = face ?: it.targetFace) else it
        }
    )
}

/** Applies an inbound ROBOT report (C.10), recording the old cell as a breadcrumb. */
fun ArenaState.withRobotAt(x: Int, y: Int, facing: Facing): ArenaState? {
    if (!Arena.isLegalRobotCentre(x, y)) return null
    val moved = robot.x != x || robot.y != y
    return copy(
        robot = RobotPose(x, y, facing),
        trail = if (moved) (trail + (robot.x to robot.y)).takeLast(MAX_TRAIL) else trail,
    )
}

fun ArenaState.cleared(): ArenaState = ArenaState()

private const val MAX_TRAIL = 64

/**
 * What the replay scrubber is showing. Empty [frames] means nothing recorded.
 */
data class ReplayState(
    val active: Boolean = false,
    val playing: Boolean = false,
    val index: Int = 0,
    val frames: List<RunFrame> = emptyList(),
) {
    val total: Int get() = frames.size
    val current: RunFrame? get() = frames.getOrNull(index)
}

// =====================================================================
// The timed runs
// =====================================================================

/**
 * Which assessed run the tablet is driving. They differ in ways the UI has to
 * know about, so this is not just a label:
 *
 *  - Task 1 gets six minutes; Task 2 gets three.
 *  - Task 1's obstacles are keyed in during the two-minute prep, in front of a
 *    supervisor. Task 2's are placed by the supervisors *after* prep and their
 *    distances are deliberately withheld, so there is nothing to key in and no
 *    map to check before starting.
 *  - Task 1 is scored on image IDs shown on the map. Task 2 is scored on time,
 *    and hitting the carpark wall disqualifies the run.
 */
enum class Task(val budgetSec: Long, val label: String) {
    TASK1(360L, "TASK 1"),
    TASK2(180L, "TASK 2"),
}

/**
 * Where a run is. The rules require the robot to stop by itself inside the
 * budget; a run that has to be stopped by hand is scored as incomplete. So the
 * phase is not cosmetic -- [OVERRUN] is the moment the attempt stopped
 * counting.
 */
enum class RunPhase { IDLE, RUNNING, FINISHED, OVERRUN }

/**
 * The state of one timed attempt.
 *
 * Deliberately pure so the whole thing is unit-testable without a device.
 */
data class RunState(
    val task: Task = Task.TASK1,
    val phase: RunPhase = RunPhase.IDLE,
    val elapsedSec: Long = 0,
    /** Obstacles carrying an image ID. */
    val identified: Int = 0,
    /** Obstacles placed on the map. */
    val placed: Int = 0,
) {
    val running: Boolean get() = phase == RunPhase.RUNNING || phase == RunPhase.OVERRUN

    /** Seconds left of the budget; negative once it is blown. */
    val remainingSec: Long get() = task.budgetSec - elapsedSec

    /**
     * The clock, counting down while running so the number on screen is the
     * one that matters. Negative time reads as `-0:12`, not `59:48`.
     */
    val clock: String
        get() = when (phase) {
            RunPhase.IDLE -> formatClock(task.budgetSec)
            else -> formatClock(remainingSec)
        }

    /** "3 / 5" -- what the supervisor is scoring in Task 1. */
    val tally: String get() = "$identified / $placed"

    /** Task 2 is scored on time, so a tally would be noise. */
    val showsTally: Boolean get() = task == Task.TASK1

    companion object {
        /** Inside this much of the budget, the clock should read as urgent. */
        const val WARN_SEC = 30L

        internal fun formatClock(seconds: Long): String {
            val sign = if (seconds < 0) "-" else ""
            val abs = kotlin.math.abs(seconds)
            return "%s%d:%02d".format(sign, abs / 60, abs % 60)
        }
    }
}

/**
 * Why a run of [task] cannot start yet, or null when it can.
 *
 * Worth checking before the press rather than after: `rpi/run_task1.py`
 * silently skips any obstacle missing a position or a face, so a forgotten
 * face is a lost image the run will never mention. The rules give no second
 * chance for a mis-keyed layout, and the supervisor is watching.
 *
 * Task 2 has no such check on purpose. Its obstacles go down after the prep
 * time and their distances are withheld, so an empty map is the correct state
 * to start from -- refusing to start would be wrong.
 */
/**
 * Does this status line mean the route runner has stopped driving?
 *
 * `rpi/run_task1.py` says so in plain words rather than a code, because the
 * same line is read by whoever is watching the Pi's terminal. Matching on the
 * words is therefore a small contract between the two files, kept here where
 * it can be tested and pointed at: **if you change the wording in
 * `run_task1.py`, change it here too.**
 *
 * Deliberately forgiving in the same way the rest of the parser is, and
 * deliberately low-stakes: a false positive stops a clock early, a false
 * negative leaves it running. Neither touches the robot.
 */
fun isRunOverNotice(text: String): Boolean {
    val t = text.lowercase()
    return "route complete" in t || "run complete" in t || "planning failed" in t
}

/**
 * Every obstacle on the map carries an image ID.
 *
 * This is the moment a Task 1 attempt stops being timed: the rules end the
 * run when the IDs are all showing on the tablet, not when the robot happens
 * to stop moving. An empty map is not "all identified" -- nothing was found,
 * so there is nothing to finish.
 */
fun ArenaState.allIdentified(): Boolean =
    obstacles.isNotEmpty() && obstacles.all { it.targetId != null }

fun ArenaState.runBlocker(task: Task): String? {
    if (task == Task.TASK2) return null
    if (obstacles.isEmpty()) return "No obstacles on the map."
    val faceless = obstacles.filter { it.targetFace == null }.map { "B${it.id}" }
    if (faceless.isNotEmpty()) {
        return "No image face set on ${faceless.joinToString(", ")}. " +
            "The planner skips obstacles without a face."
    }
    return null
}
