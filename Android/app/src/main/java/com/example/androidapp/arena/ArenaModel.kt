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

    /** The start zone is the 3 x 3 block at the origin corner. */
    const val START_ZONE_SPAN = 3

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
