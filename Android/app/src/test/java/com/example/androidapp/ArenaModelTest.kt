package com.example.androidapp

import com.example.androidapp.arena.Arena
import com.example.androidapp.arena.ArenaState
import com.example.androidapp.arena.Facing
import com.example.androidapp.arena.withObstacleAdded
import com.example.androidapp.arena.withObstacleMoved
import com.example.androidapp.arena.withObstacleRemoved
import com.example.androidapp.arena.withRobotAt
import com.example.androidapp.arena.withTargetReported
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

class ArenaModelTest {

    private fun stateWith(vararg cells: Pair<Int, Int>): ArenaState {
        var s = ArenaState()
        cells.forEach { (x, y) -> s = s.withObstacleAdded(x, y)!!.first }
        return s
    }

    // --- numbering, C.6 --------------------------------------------------

    @Test fun `obstacles number from one upward`() {
        val s = stateWith(1 to 1, 2 to 2, 3 to 3)
        assertEquals(listOf(1, 2, 3), s.obstacles.map { it.id })
    }

    @Test fun `deleting does not renumber the survivors`() {
        // The robot has already been told about B3 by number; renaming it would
        // silently point the path planner at the wrong block.
        val s = stateWith(1 to 1, 2 to 2, 3 to 3).withObstacleRemoved(2)
        assertEquals(listOf(1, 3), s.obstacles.map { it.id })
    }

    @Test fun `a freed number is reused by the next obstacle`() {
        val s = stateWith(1 to 1, 2 to 2, 3 to 3).withObstacleRemoved(2)
        val (next, created) = s.withObstacleAdded(5, 5)!!
        assertEquals(2, created.id)
        assertEquals(listOf(1, 3, 2), next.obstacles.map { it.id })
    }

    @Test fun `two obstacles cannot share a cell`() {
        assertNull(stateWith(4 to 4).withObstacleAdded(4, 4))
    }

    @Test fun `obstacles cannot be placed outside the arena`() {
        assertNull(ArenaState().withObstacleAdded(-1, 0))
        assertNull(ArenaState().withObstacleAdded(0, Arena.SIZE))
    }

    @Test fun `an obstacle cannot be dragged onto another`() {
        assertNull(stateWith(1 to 1, 2 to 2).withObstacleMoved(1, 2, 2))
    }

    @Test fun `an obstacle can be dragged onto its own cell`() {
        assertNotNull(stateWith(1 to 1).withObstacleMoved(1, 1, 1))
    }

    // --- robot, C.10 -----------------------------------------------------

    @Test fun `robot centres must leave room for a three by three footprint`() {
        assertNull(ArenaState().withRobotAt(0, 0, Facing.N))
        assertNull(ArenaState().withRobotAt(19, 19, Facing.N))
        assertNotNull(ArenaState().withRobotAt(1, 1, Facing.N))
        assertNotNull(ArenaState().withRobotAt(18, 18, Facing.N))
    }

    @Test fun `moving the robot leaves a breadcrumb`() {
        val s = ArenaState().withRobotAt(5, 5, Facing.E)!!
        assertEquals(listOf(1 to 1), s.trail)
    }

    @Test fun `turning on the spot does not leave a breadcrumb`() {
        val s = ArenaState().withRobotAt(1, 1, Facing.E)!!
        assertEquals(emptyList<Pair<Int, Int>>(), s.trail)
    }

    // --- targets, C.9 ----------------------------------------------------

    @Test fun `target report lands on the right obstacle`() {
        val s = stateWith(4 to 4, 6 to 6).withTargetReported(2, 11, Facing.N)!!
        assertEquals(11, s.obstacle(2)!!.targetId)
        assertEquals(Facing.N, s.obstacle(2)!!.targetFace)
        assertNull(s.obstacle(1)!!.targetId)
    }

    @Test fun `target report for an unknown obstacle is refused`() {
        assertNull(stateWith(4 to 4).withTargetReported(9, 11, null))
    }

    @Test fun `target id outside the image pool is refused`() {
        assertNull(stateWith(4 to 4).withTargetReported(1, 7, null))
        assertNull(stateWith(4 to 4).withTargetReported(1, 41, null))
    }

    @Test fun `a target report without a face keeps the face we annotated`() {
        val annotated = stateWith(4 to 4).let {
            it.copy(obstacles = it.obstacles.map { o -> o.copy(targetFace = Facing.S) })
        }
        val s = annotated.withTargetReported(1, 11, null)!!
        assertEquals(Facing.S, s.obstacle(1)!!.targetFace)
    }

    // --- image pool glyphs ------------------------------------------------

    @Test fun `every id in the pool has a glyph`() {
        Arena.TARGET_ID_RANGE.forEach {
            assertNotNull("no glyph for $it", Arena.glyphFor(it))
        }
    }

    @Test fun `glyphs match the briefing image pool`() {
        assertEquals("1", Arena.glyphFor(11))
        assertEquals("9", Arena.glyphFor(19))
        assertEquals("A", Arena.glyphFor(20))
        assertEquals("H", Arena.glyphFor(27))
        assertEquals("S", Arena.glyphFor(28))
        assertEquals("Z", Arena.glyphFor(35))
        assertEquals("↑", Arena.glyphFor(36))
        assertEquals("←", Arena.glyphFor(39))
        assertEquals("●", Arena.glyphFor(40))
    }

    @Test fun `ids outside the pool have no glyph`() {
        assertNull(Arena.glyphFor(10))
        assertNull(Arena.glyphFor(41))
        assertNull(Arena.glyphFor(0))
    }

    @Test fun `no two ids share a glyph`() {
        val glyphs = Arena.TARGET_ID_RANGE.mapNotNull { Arena.glyphFor(it) }
        assertEquals(glyphs.size, glyphs.toSet().size)
    }

    // --- facing ----------------------------------------------------------

    @Test fun `bearings run clockwise from north`() {
        assertEquals(0f, Facing.N.bearingDeg)
        assertEquals(90f, Facing.E.bearingDeg)
        assertEquals(180f, Facing.S.bearingDeg)
        assertEquals(270f, Facing.W.bearingDeg)
    }

    @Test fun `turning left four times returns to the start`() {
        var f = Facing.N
        repeat(4) { f = f.turnedLeft() }
        assertEquals(Facing.N, f)
    }

    @Test fun `north turned right is east`() {
        assertEquals(Facing.E, Facing.N.turnedRight())
    }
}
