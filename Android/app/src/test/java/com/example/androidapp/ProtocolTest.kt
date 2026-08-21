package com.example.androidapp

import com.example.androidapp.arena.Facing
import com.example.androidapp.protocol.Inbound
import com.example.androidapp.protocol.Outbound
import com.example.androidapp.protocol.parseInbound
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The parser is the one place where a supervisor's typing meets our code, so
 * it is the one place worth testing exhaustively. These run on the JVM in
 * about a second - no emulator, no tablet, no AMD tool.
 */
class ProtocolTest {

    // --- ROBOT, C.10 -----------------------------------------------------

    @Test fun `robot in the documented form`() {
        assertEquals(Inbound.Robot(7, 2, Facing.W), parseInbound("ROBOT,7,2,W"))
    }

    @Test fun `robot tolerates spaces`() {
        assertEquals(Inbound.Robot(7, 2, Facing.W), parseInbound("  ROBOT , 7 , 2 , W  "))
    }

    @Test fun `robot tolerates lower case`() {
        assertEquals(Inbound.Robot(0, 19, Facing.N), parseInbound("robot,0,19,n"))
    }

    @Test fun `robot tolerates a trailing comma`() {
        assertEquals(Inbound.Robot(3, 4, Facing.S), parseInbound("ROBOT,3,4,S,"))
    }

    @Test fun `robot with a bad direction is unknown, not a crash`() {
        assertTrue(parseInbound("ROBOT,3,4,Q") is Inbound.Unknown)
    }

    @Test fun `robot with missing fields is unknown`() {
        assertTrue(parseInbound("ROBOT,3,4") is Inbound.Unknown)
    }

    // --- TARGET, C.9 -----------------------------------------------------

    @Test fun `target in the briefing slide form`() {
        assertEquals(Inbound.Target(2, 11, null), parseInbound("TARGET,B2,11"))
    }

    @Test fun `target in the written checklist form`() {
        // The checklist writes it without the B prefix and with spaces.
        assertEquals(Inbound.Target(2, 11, null), parseInbound("TARGET, 2, 11"))
    }

    @Test fun `target with a face`() {
        assertEquals(Inbound.Target(2, 11, Facing.N), parseInbound("TARGET,B2,11,N"))
    }

    @Test fun `target with a lower case prefix`() {
        assertEquals(Inbound.Target(13, 40, Facing.E), parseInbound("target,b13,40,e"))
    }

    @Test fun `target with junk id is unknown`() {
        assertTrue(parseInbound("TARGET,B2,eleven") is Inbound.Unknown)
    }

    // --- MSG, C.4 --------------------------------------------------------

    @Test fun `message unwraps the brackets`() {
        assertEquals(Inbound.Message("Moving"), parseInbound("MSG,[Moving]"))
    }

    @Test fun `message without brackets`() {
        assertEquals(Inbound.Message("looking for target 2"), parseInbound("MSG,looking for target 2"))
    }

    // --- the Pi bridge's own vocabulary, from rpi/a1_bridge.py -----------

    @Test fun `bridge greeting is a plain message`() {
        assertEquals(Inbound.Message("RPi bridge ready"), parseInbound("STATUS,RPi bridge ready"))
    }

    @Test fun `bridge forwarding receipt is not a plain message`() {
        assertEquals(Inbound.Forwarded("FW010"), parseInbound("STATUS,SENT,FW010"))
    }

    @Test fun `stm replies come through as stm replies`() {
        listOf("READY", "DONE", "STALL", "TIMEOUT", "ACK", "BUSY", "ERR", "NO_REPLY").forEach {
            assertEquals("failed on $it", Inbound.StmReply(it), parseInbound("STM,$it"))
        }
    }

    @Test fun `stm reply is upper cased so the view model can match on it`() {
        assertEquals(Inbound.StmReply("DONE"), parseInbound("stm,done"))
    }

    @Test fun `bridge rejection is surfaced, not swallowed`() {
        assertEquals(Inbound.Rejected("INVALID_COMMAND"), parseInbound("ERR,INVALID_COMMAND"))
    }

    @Test fun `bare STM with no reply is not mistaken for a reply`() {
        assertTrue(parseInbound("STM,") is Inbound.Unknown)
    }

    // --- junk ------------------------------------------------------------

    @Test fun `garbage never throws`() {
        val junk = listOf(
            "",
            "   ",
            ",,,",
            " ",
            "\t\t",
            "ROBOT",
            "TARGET",
            "!!!",
            "ROBOT,,,,",
            "TARGET,,",
            "😀",
        )
        junk.forEach { assertTrue("failed on <$it>", parseInbound(it) is Inbound.Unknown) }
    }

    // --- outbound --------------------------------------------------------

    @Test fun `outbound map strings match the briefing`() {
        assertEquals("ADD,B1,(10,6)", Outbound.add(1, 10, 6))
        assertEquals("SUB,B1", Outbound.sub(1))
        assertEquals("FACE,B2,N", Outbound.face(2, Facing.N))
    }

    @Test fun `motion commands are zero padded to three digits`() {
        assertEquals("FW010", Outbound.forward(10))
        assertEquals("BW100", Outbound.backward(100))
        assertEquals("FL090", Outbound.forwardLeft(90))
        assertEquals("BR360", Outbound.backRight(360))
    }

    @Test fun `our own outbound strings are not mistaken for inbound ones`() {
        assertTrue(parseInbound(Outbound.add(4, 12, 15)) is Inbound.Unknown)
        assertTrue(parseInbound(Outbound.sub(4)) is Inbound.Unknown)
        assertTrue(parseInbound(Outbound.face(4, Facing.W)) is Inbound.Unknown)
    }
}
