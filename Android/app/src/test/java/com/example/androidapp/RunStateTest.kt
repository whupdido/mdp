package com.example.androidapp

import com.example.androidapp.arena.ArenaState
import com.example.androidapp.arena.Facing
import com.example.androidapp.arena.RunPhase
import com.example.androidapp.arena.RunState
import com.example.androidapp.arena.Task
import com.example.androidapp.arena.allIdentified
import com.example.androidapp.arena.runBlocker
import com.example.androidapp.arena.withObstacleAdded
import com.example.androidapp.arena.withTargetFace
import com.example.androidapp.arena.withTargetReported
import com.example.androidapp.protocol.Outbound
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The timed-run rules, checked against the numbers in the rules PDF rather
 * than against whatever the code happens to do.
 */
class RunStateTest {

    // --- the budgets -----------------------------------------------------

    @Test fun `task 1 gets six minutes and task 2 gets three`() {
        assertEquals(360L, Task.TASK1.budgetSec)
        assertEquals(180L, Task.TASK2.budgetSec)
    }

    @Test fun `an idle panel shows the budget for the chosen task`() {
        assertEquals("6:00", RunState(task = Task.TASK1).clock)
        assertEquals("3:00", RunState(task = Task.TASK2).clock)
    }

    // --- the clock -------------------------------------------------------

    @Test fun `the clock counts down, not up`() {
        assertEquals("5:00", RunState(phase = RunPhase.RUNNING, elapsedSec = 60).clock)
        assertEquals("0:59", RunState(phase = RunPhase.RUNNING, elapsedSec = 301).clock)
        assertEquals("0:00", RunState(phase = RunPhase.RUNNING, elapsedSec = 360).clock)
    }

    @Test fun `task 2 counts down from its own shorter budget`() {
        val run = RunState(task = Task.TASK2, phase = RunPhase.RUNNING, elapsedSec = 60)
        assertEquals("2:00", run.clock)
        assertEquals(120L, run.remainingSec)
    }

    @Test fun `overrun reads as negative, not as fifty nine minutes`() {
        // The bug this pins: 360-372 = -12, and a naive mm:ss of a negative
        // number prints 59:48, which looks like time remaining.
        val run = RunState(phase = RunPhase.OVERRUN, elapsedSec = 372)
        assertEquals("-0:12", run.clock)
        assertEquals(-12L, run.remainingSec)
    }

    @Test fun `overrun past a minute still reads negative`() {
        assertEquals("-1:30", RunState(phase = RunPhase.OVERRUN, elapsedSec = 450).clock)
    }

    @Test fun `task 2 overruns three minutes early, not six`() {
        val run = RunState(task = Task.TASK2, phase = RunPhase.OVERRUN, elapsedSec = 190)
        assertEquals("-0:10", run.clock)
    }

    // --- phases ----------------------------------------------------------

    @Test fun `only running phases count as running`() {
        assertFalse(RunState(phase = RunPhase.IDLE).running)
        assertTrue(RunState(phase = RunPhase.RUNNING).running)
        assertTrue(RunState(phase = RunPhase.OVERRUN).running)
        assertFalse(RunState(phase = RunPhase.FINISHED).running)
    }

    @Test fun `the tally is what the supervisor scores in task 1 only`() {
        assertEquals("3 / 5", RunState(identified = 3, placed = 5).tally)
        assertTrue(RunState(task = Task.TASK1).showsTally)
        assertFalse(RunState(task = Task.TASK2).showsTally)
    }

    // --- the start string ------------------------------------------------

    @Test fun `each task has its own start string`() {
        assertEquals("START", Outbound.start(Task.TASK1))
        assertEquals("START2", Outbound.start(Task.TASK2))
    }

    // --- the pre-flight check -------------------------------------------
    //
    // rpi/run_task1.py drops any obstacle with no face, without saying so.
    // The rules give no second chance for a mis-keyed layout, so the app has
    // to catch it before the press, not after.

    private fun mapOf(vararg cells: Pair<Int, Int>): ArenaState {
        var s = ArenaState()
        cells.forEach { (x, y) -> s = s.withObstacleAdded(x, y)!!.first }
        return s
    }

    @Test fun `an empty map cannot start task 1`() {
        assertNotNull(ArenaState().runBlocker(Task.TASK1))
    }

    @Test fun `an obstacle with no face blocks the start and is named`() {
        val blocker = mapOf(5 to 13, 5 to 7).runBlocker(Task.TASK1)
        assertNotNull(blocker)
        assertTrue("should name B1: $blocker", blocker!!.contains("B1"))
        assertTrue("should name B2: $blocker", blocker.contains("B2"))
    }

    @Test fun `naming is limited to the obstacles actually missing a face`() {
        val blocker = mapOf(5 to 13, 5 to 7)
            .withTargetFace(1, Facing.W)
            .runBlocker(Task.TASK1)
        assertNotNull(blocker)
        assertFalse("B1 has a face, should not be named: $blocker", blocker!!.contains("B1"))
        assertTrue("B2 has none: $blocker", blocker.contains("B2"))
    }

    @Test fun `a fully annotated map is ready to run`() {
        // The five-obstacle layout from the rules PDF, all faces set.
        val ready = mapOf(5 to 13, 5 to 7, 12 to 9, 15 to 15, 15 to 4)
            .withTargetFace(1, Facing.W)
            .withTargetFace(2, Facing.S)
            .withTargetFace(3, Facing.E)
            .withTargetFace(4, Facing.S)
            .withTargetFace(5, Facing.N)
        assertNull(ready.runBlocker(Task.TASK1))
    }

    // --- when the attempt is over ---------------------------------------
    //
    // The rules stop the clock when the image IDs are all showing on the
    // tablet, not when the robot stops moving.

    @Test fun `an empty map has not identified everything`() {
        // Otherwise a run with no obstacles would report itself complete the
        // instant it started.
        assertFalse(ArenaState().allIdentified())
    }

    @Test fun `a map is not done until every obstacle has an id`() {
        val two = mapOf(5 to 13, 5 to 7)
        assertFalse(two.allIdentified())
        val one = two.withTargetReported(1, 35, Facing.W)!!
        assertFalse("one of two is not done", one.allIdentified())
        val both = one.withTargetReported(2, 17, Facing.S)!!
        assertTrue("both reported", both.allIdentified())
    }

    @Test fun `task 2 starts from an empty map, because that is correct`() {
        // Task 2's obstacles are placed by the supervisors after the prep
        // time and their distances are withheld. Refusing to start with an
        // empty map would block the only legal way to run it.
        assertNull(ArenaState().runBlocker(Task.TASK2))
        assertNull(mapOf(5 to 13).runBlocker(Task.TASK2))
    }
}
