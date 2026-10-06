package com.example.androidapp

import com.example.androidapp.arena.ArenaState
import com.example.androidapp.arena.Facing
import com.example.androidapp.arena.RunPhase
import com.example.androidapp.arena.RunState
import com.example.androidapp.arena.Task
import com.example.androidapp.arena.allIdentified
import com.example.androidapp.arena.isRunOverNotice
import com.example.androidapp.arena.runBlocker
import com.example.androidapp.arena.startsInCarpark
import com.example.androidapp.arena.withStartPose
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

    // --- recognising that the route runner has stopped -------------------
    //
    // A small contract with rpi/run_task1.py: it reports in plain words, and
    // the app stops the clock on them. Pinned so changing the wording on one
    // side without the other shows up here.

    @Test fun `the route runner's own completion lines are recognised`() {
        // Verbatim from rpi/run_task1.py.
        assertTrue(isRunOverNotice("Task 1 route complete"))
        assertTrue(isRunOverNotice("Planning failed, check RPi logs"))
    }

    @Test fun `recognition is case-insensitive like the rest of the parser`() {
        assertTrue(isRunOverNotice("TASK 1 ROUTE COMPLETE"))
    }

    @Test fun `ordinary status chatter does not stop the clock`() {
        listOf(
            "RPi bridge ready",
            "Task1 runner ready",
            "Run starting",
            "Reached obstacle 3, capturing",
            "Robot ready.",
        ).forEach { assertFalse("should not end the run: $it", isRunOverNotice(it)) }
    }

    // --- START is gated on having a route --------------------------------
    //
    // Task 1 is three presses -- SETUP, PLAN, START -- and only one is live
    // at a time, so they cannot be taken out of order under time pressure.

    @Test fun `exactly one task 1 press is live in each phase`() {
        data class Expect(val phase: RunPhase, val setup: Boolean, val plan: Boolean, val start: Boolean)
        listOf(
            Expect(RunPhase.IDLE, setup = true, plan = false, start = false),
            Expect(RunPhase.COMPUTING, setup = false, plan = false, start = false),
            Expect(RunPhase.PLANNED, setup = true, plan = true, start = false),
            Expect(RunPhase.ARMING, setup = false, plan = false, start = false),
            Expect(RunPhase.ARMED, setup = true, plan = false, start = true),
        ).forEach { e ->
            val run = RunState(phase = e.phase)
            assertEquals("setup on ${e.phase}", e.setup, run.canSetup)
            assertEquals("plan on ${e.phase}", e.plan, run.canPlan)
            assertEquals("start on ${e.phase}", e.start, run.canStart)
        }
    }

    @Test fun `start is dead until the robot has been checked`() {
        // PLANNED means a route exists but nothing has verified the board is
        // awake or the laptop is reachable. That is what PLAN is for.
        assertFalse(RunState(phase = RunPhase.PLANNED).canStart)
        assertTrue(RunState(phase = RunPhase.ARMED).canStart)
    }

    @Test fun `nothing is pressable while the Pi is working`() {
        assertTrue(RunState(phase = RunPhase.COMPUTING).busy)
        assertTrue(RunState(phase = RunPhase.ARMING).busy)
        assertFalse(RunState(phase = RunPhase.PLANNED).busy)
        assertFalse(RunState(phase = RunPhase.ARMED).busy)
    }

    @Test fun `task 2 can start straight away, having nothing to plan`() {
        // Its obstacles are not placed until after the preparation time, so
        // there is no route to compute and gating START would be wrong.
        assertTrue(RunState(task = Task.TASK2, phase = RunPhase.IDLE).canStart)
        assertFalse("and it has no SETUP press at all",
            RunState(task = Task.TASK2, phase = RunPhase.IDLE).canSetup)
    }

    @Test fun `the clock shows the full budget until the run actually starts`() {
        // All three presses happen in the preparation window, off the clock.
        listOf(RunPhase.IDLE, RunPhase.COMPUTING, RunPhase.PLANNED, RunPhase.ARMING, RunPhase.ARMED)
            .forEach { assertEquals("failed on $it", "6:00", RunState(phase = it).clock) }
    }

    // --- the start pose is editable --------------------------------------
    //
    // The planner is told where we are parked rather than assuming (1,1,N):
    // which carpark cell, facing which way, is the supervisor's call.

    @Test fun `the start pose can be moved and turned`() {
        val moved = ArenaState().withStartPose(2, 2, Facing.E)
        assertNotNull(moved)
        assertEquals(2, moved!!.robot.x)
        assertEquals(Facing.E, moved.robot.facing)
    }

    @Test fun `moving the start pose leaves no trail`() {
        // A trail would claim the robot drove there. It did not; this is an
        // edit to the plan, made before anything moves.
        val moved = ArenaState().withStartPose(5, 5, Facing.N)!!
        assertTrue(moved.trail.isEmpty())
    }

    @Test fun `the start pose cannot be put where the robot would not fit`() {
        // Its body is 3 x 3, so a centre on the boundary hangs off the arena.
        assertNull(ArenaState().withStartPose(0, 0, Facing.N))
        assertNull(ArenaState().withStartPose(19, 19, Facing.N))
    }

    @Test fun `the start pose cannot sit on an obstacle`() {
        val withBlock = mapOf(5 to 5)
        assertNull("robot body would cover B1", withBlock.withStartPose(5, 5, Facing.N))
        assertNull("still covers it one cell away", withBlock.withStartPose(6, 6, Facing.N))
        assertNotNull("clear of it here", withBlock.withStartPose(9, 9, Facing.N))
    }

    @Test fun `the carpark check wants the whole body inside`() {
        // Leaving the carpark during preparation is a disqualification
        // (FAQ 9), so this is worth saying before the press.
        assertTrue(ArenaState().startsInCarpark())                       // (1,1)
        assertTrue(ArenaState().withStartPose(2, 2, Facing.N)!!.startsInCarpark())
        assertFalse(
            "centre at (3,3) puts a third of the body outside the 4x4 zone",
            ArenaState().withStartPose(3, 3, Facing.N)!!.startsInCarpark(),
        )
        assertFalse(ArenaState().withStartPose(9, 9, Facing.N)!!.startsInCarpark())
    }

    @Test fun `task 2 starts from an empty map, because that is correct`() {
        // Task 2's obstacles are placed by the supervisors after the prep
        // time and their distances are withheld. Refusing to start with an
        // empty map would block the only legal way to run it.
        assertNull(ArenaState().runBlocker(Task.TASK2))
        assertNull(mapOf(5 to 13).runBlocker(Task.TASK2))
    }
}
