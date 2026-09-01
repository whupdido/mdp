package com.example.androidapp.arena

import android.animation.ValueAnimator
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RadialGradient
import android.graphics.RectF
import android.graphics.Shader
import android.util.AttributeSet
import android.util.TypedValue
import android.view.MotionEvent
import android.view.View
import android.view.ViewConfiguration
import android.view.animation.PathInterpolator
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sin

/**
 * The arena canvas. C.5 draws it, C.6 and C.7 make it interactive, C.9 and
 * C.10 are driven by whatever the view model feeds into [state].
 *
 * The view knows how to draw a grid and which cell a finger is over. It does
 * not know what a Bluetooth message is, and it decides nothing — every touch
 * that means something is handed upward through a callback.
 *
 * Two things here are deliberate rather than decorative:
 *
 *  - Obstacles are drawn as raised blocks with a lit top and a cast shadow,
 *    because they *are* 10 cm cubes standing on the floor. It stays a true
 *    plan view — no fake isometry — so coordinates still read correctly.
 *
 *  - The robot does not teleport between poses. It travels, and because the
 *    car cannot turn on the spot (Ackermann steering, see STM32_motion_spec)
 *    a turn leaves along the heading the robot was already facing and curves
 *    into the new one.
 */
class ArenaView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0,
) : View(context, attrs, defStyle) {

    // --- callbacks ------------------------------------------------------

    var onAddObstacle: (x: Int, y: Int) -> Unit = { _, _ -> }
    var onMoveObstacle: (id: Int, x: Int, y: Int) -> Unit = { _, _, _ -> }
    var onRemoveObstacle: (id: Int) -> Unit = { }
    var onSetTargetFace: (id: Int, face: Facing) -> Unit = { _, _ -> }

    /** Cell under the finger. For the readout chip. */
    var onHover: (x: Int, y: Int) -> Unit = { _, _ -> }
    var onHoverEnd: () -> Unit = { }

    // --- state ----------------------------------------------------------

    var state: ArenaState = ArenaState()
        set(value) {
            val previous = field
            field = value
            if (selectorFor != null && value.obstacle(selectorFor!!) == null) selectorFor = null
            if (previous.robot != value.robot) animateRobotTo(value.robot) else invalidate()
        }

    /**
     * When set, the view draws this pose instead of [state]'s. Used by replay
     * so scrubbing does not disturb the live model.
     */
    var replayPose: RobotPose? = null
        set(value) {
            field = value
            if (value != null) {
                animator?.cancel()
                poseX = value.x.toFloat(); poseY = value.y.toFloat()
                poseBearing = value.facing.bearingDeg
            }
            invalidate()
        }

    /** Trail cells to draw. Replay overrides the live trail while scrubbing. */
    var replayTrail: List<Pair<Int, Int>>? = null
        set(value) { field = value; invalidate() }

    private var selectorFor: Int? = null

    private var dragId: Int? = null
    private var dragging = false
    private var dragX = 0f
    private var dragY = 0f
    private var downX = 0f
    private var downY = 0f
    private val slop = ViewConfiguration.get(context).scaledTouchSlop

    // --- animated robot pose --------------------------------------------

    private var poseX = state.robot.x.toFloat()
    private var poseY = state.robot.y.toFloat()
    private var poseBearing = state.robot.facing.bearingDeg

    private var animator: ValueAnimator? = null
    private val travelEase = PathInterpolator(0.33f, 0f, 0.15f, 1f)

    // --- geometry -------------------------------------------------------

    private var cell = 0f
    private var gridLeft = 0f
    private var gridTop = 0f
    private var gridBottom = 0f
    private var gutter = 0f

    private fun dp(v: Float) = v * resources.displayMetrics.density
    private fun sp(v: Float) =
        TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_SP, v, resources.displayMetrics)

    // --- paint ----------------------------------------------------------

    private val boardPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = BOARD }
    private val vignettePaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = GRID; style = Paint.Style.STROKE; strokeWidth = dp(0.5f)
    }
    private val majorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = GRID_MAJOR; style = Paint.Style.STROKE; strokeWidth = dp(0.9f)
    }
    private val framePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = FRAME; style = Paint.Style.STROKE; strokeWidth = dp(1.6f)
    }
    private val startFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = START_FILL }
    private val startHatch = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = START_HATCH; style = Paint.Style.STROKE; strokeWidth = dp(1f)
    }
    private val axisPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = AXIS; textAlign = Paint.Align.CENTER
    }

    private val shadowPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = BLOCK_SHADOW }
    private val blockSide = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = BLOCK_SIDE }
    private val blockTop = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = BLOCK_TOP }
    private val blockEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = BLOCK_EDGE; style = Paint.Style.STROKE; strokeWidth = dp(0.9f)
    }
    private val facePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = TARGET }
    private val faceGlowPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = TARGET_GLOW }
    private val idPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }
    private val glyphChipPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = TARGET }
    private val glyphRingPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = BOARD_RING }
    private val glyphPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }

    private val robotBody = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = ROBOT_BODY }
    private val robotEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ROBOT_EDGE; style = Paint.Style.STROKE; strokeWidth = dp(1.4f)
    }
    private val robotGlow = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = ROBOT_GLOW }
    private val nosePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = ROBOT_NOSE }
    private val trailPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeCap = Paint.Cap.ROUND; strokeJoin = Paint.Join.ROUND
    }

    private val highlightPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ACCENT; style = Paint.Style.STROKE; strokeWidth = dp(2f)
    }
    private val dangerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = DANGER }
    private val scrimPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = SCRIM }
    private val quadrantPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = QUADRANT }
    private val quadrantEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = QUADRANT_EDGE; style = Paint.Style.STROKE; strokeWidth = dp(1.4f)
    }
    private val quadrantLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }

    private val rect = RectF()
    private val rect2 = RectF()
    private val path = Path()

    // -----------------------------------------------------------------
    // Measurement — the arena is always square
    // -----------------------------------------------------------------

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        val h = MeasureSpec.getSize(heightMeasureSpec)
        val side = min(w, h).coerceAtLeast(1)
        setMeasuredDimension(side, side)
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        gutter = dp(18f)
        val usable = min(w - gutter, h - gutter)
        cell = usable / Arena.SIZE
        gridLeft = gutter
        gridTop = 0f
        gridBottom = gridTop + cell * Arena.SIZE
        axisPaint.textSize = min(sp(10f), cell * 0.62f)
        quadrantLabel.textSize = dp(15f)
        trailPaint.strokeWidth = cell * 0.16f

        val cx = gridLeft + cell * Arena.SIZE / 2f
        val cy = gridTop + cell * Arena.SIZE / 2f
        vignettePaint.shader = RadialGradient(
            cx, cy, cell * Arena.SIZE * 0.72f,
            intArrayOf(0x00000000, VIGNETTE), floatArrayOf(0.55f, 1f), Shader.TileMode.CLAMP,
        )
    }

    // -----------------------------------------------------------------
    // Robot travel
    // -----------------------------------------------------------------

    /**
     * Eases the drawn robot from where it currently appears to its new pose.
     *
     * The path is a quadratic curve whose control point sits ahead of the old
     * position along the old heading. That is what gives the motion its
     * Ackermann character: the car pulls away in the direction it was already
     * pointing and arcs into the new heading, instead of sliding sideways or
     * spinning on the spot — neither of which this chassis can do.
     */
    private fun animateRobotTo(target: RobotPose) {
        if (replayPose != null) { invalidate(); return }
        animator?.cancel()

        val x0 = poseX
        val y0 = poseY
        val b0 = poseBearing
        val x1 = target.x.toFloat()
        val y1 = target.y.toFloat()
        val b1 = nearestEquivalentAngle(b0, target.facing.bearingDeg)

        val distance = hypot(x1 - x0, y1 - y0)

        // Lead distance ahead of the old heading. No lead for a move with no
        // turn — a straight line is the honest picture there.
        //
        // Left and right are not symmetric. Kush re-measured the turn radii on
        // 31-Aug-2026 (stm32/STM32_motion_spec.md): FL 317 mm against FR
        // 413 mm, so a right turn genuinely swings about 30 % wider than a
        // left. Drawing them the same would misrepresent the one asymmetry
        // that actually costs us space in the arena.
        val turning = distance > 0.01f && abs(b1 - b0) > 1f
        val turningRight = b1 > b0
        val lead = when {
            !turning -> 0f
            turningRight -> distance * LEAD_LEFT * (RADIUS_FR_MM / RADIUS_FL_MM)
            else -> distance * LEAD_LEFT
        }
        val rad = Math.toRadians(b0.toDouble())
        val ctrlX = x0 + (sin(rad) * lead).toFloat()
        val ctrlY = y0 + (cos(rad) * lead).toFloat()

        animator = ValueAnimator.ofFloat(0f, 1f).apply {
            duration = (220L + distance * 45L).toLong().coerceAtMost(900L)
            interpolator = travelEase
            addUpdateListener { a ->
                val t = a.animatedValue as Float
                val inv = 1f - t
                if (lead > 0f) {
                    poseX = inv * inv * x0 + 2f * inv * t * ctrlX + t * t * x1
                    poseY = inv * inv * y0 + 2f * inv * t * ctrlY + t * t * y1
                } else {
                    poseX = x0 + (x1 - x0) * t
                    poseY = y0 + (y1 - y0) * t
                }
                poseBearing = b0 + (b1 - b0) * t
                invalidate()
            }
            start()
        }
    }

    /** Picks the representation of [to] that is the shortest turn away from [from]. */
    private fun nearestEquivalentAngle(from: Float, to: Float): Float {
        var delta = (to - from) % 360f
        if (delta > 180f) delta -= 360f
        if (delta < -180f) delta += 360f
        return from + delta
    }

    override fun onDetachedFromWindow() {
        animator?.cancel()
        super.onDetachedFromWindow()
    }

    // -----------------------------------------------------------------
    // Drawing
    // -----------------------------------------------------------------

    override fun onDraw(canvas: Canvas) {
        if (cell <= 0f) return
        drawBoard(canvas)
        drawAxes(canvas)
        drawTrail(canvas)
        drawObstacles(canvas)
        drawRobot(canvas)
        drawDragGhost(canvas)
        drawFaceSelector(canvas)
    }

    private fun drawBoard(canvas: Canvas) {
        val right = gridLeft + cell * Arena.SIZE
        rect.set(gridLeft, gridTop, right, gridBottom)
        canvas.drawRect(rect, boardPaint)

        // Start zone: tinted, with hatching so it reads as a marked area on the
        // floor rather than just another coloured square.
        val zone = cell * Arena.START_ZONE_SPAN
        rect.set(gridLeft, gridBottom - zone, gridLeft + zone, gridBottom)
        canvas.drawRect(rect, startFill)
        canvas.save()
        canvas.clipRect(rect)
        var h = -zone
        while (h < zone * 2) {
            canvas.drawLine(gridLeft + h, gridBottom, gridLeft + h + zone, gridBottom - zone, startHatch)
            h += dp(9f)
        }
        canvas.restore()

        for (i in 0..Arena.SIZE) {
            val paint = if (i % 5 == 0) majorPaint else gridPaint
            val x = gridLeft + i * cell
            canvas.drawLine(x, gridTop, x, gridBottom, paint)
            val y = gridTop + i * cell
            canvas.drawLine(gridLeft, y, right, y, paint)
        }

        canvas.drawRect(gridLeft, gridTop, right, gridBottom, vignettePaint)
        canvas.drawRect(gridLeft, gridTop, right, gridBottom, framePaint)
    }

    private fun drawAxes(canvas: Canvas) {
        val step = if (cell >= dp(16f)) 1 else 2
        val baseline = gridBottom + axisPaint.textSize + dp(3f)
        for (i in 0 until Arena.SIZE step step) {
            canvas.drawText(i.toString(), cellCentreX(i), baseline, axisPaint)
        }
        axisPaint.textAlign = Paint.Align.RIGHT
        for (i in 0 until Arena.SIZE step step) {
            canvas.drawText(
                i.toString(), gridLeft - dp(4f),
                cellCentreY(i) + axisPaint.textSize * 0.36f, axisPaint,
            )
        }
        axisPaint.textAlign = Paint.Align.CENTER
    }

    /** Where the robot has been, fading out towards the oldest point. */
    private fun drawTrail(canvas: Canvas) {
        val cells = replayTrail ?: state.trail
        if (cells.isEmpty()) return
        val points = cells.map { cellCentreX(it.first) to cellCentreY(it.second) } +
            (cellCentreX2(poseX) to cellCentreY2(poseY))
        for (i in 0 until points.size - 1) {
            val fade = (i + 1f) / points.size
            trailPaint.color = withAlpha(TRAIL, (28 + 150 * fade).toInt())
            trailPaint.strokeWidth = cell * (0.07f + 0.09f * fade)
            canvas.drawLine(
                points[i].first, points[i].second,
                points[i + 1].first, points[i + 1].second, trailPaint,
            )
        }
    }

    private fun drawObstacles(canvas: Canvas) {
        state.obstacles.forEach { o ->
            if (dragging && o.id == dragId) return@forEach
            drawObstacle(canvas, o, cellLeft(o.x), cellTop(o.y), cell)
        }
    }

    /**
     * A 10 cm cube seen from above: a cast shadow, a darker rim for the sides
     * catching less light, and a lit top face. Still a true plan view.
     */
    private fun drawObstacle(canvas: Canvas, o: Obstacle, left: Float, top: Float, size: Float) {
        val r = size * 0.14f

        // Cast shadow, offset down-right as if lit from the front-left.
        rect.set(left + size * 0.10f, top + size * 0.12f, left + size * 1.02f, top + size * 1.04f)
        canvas.drawRoundRect(rect, r, r, shadowPaint)

        // Sides.
        rect.set(left + size * 0.03f, top + size * 0.03f, left + size * 0.97f, top + size * 0.97f)
        canvas.drawRoundRect(rect, r, r, blockSide)

        // Lit top face, inset so a sliver of the side shows.
        rect2.set(
            left + size * 0.10f, top + size * 0.08f,
            left + size * 0.90f, top + size * 0.86f,
        )
        canvas.drawRoundRect(rect2, r * 0.8f, r * 0.8f, blockTop)
        canvas.drawRoundRect(rect, r, r, blockEdge)

        // C.7 / C.9 — the face holding the target image, with a soft glow so it
        // is visible across the room and on camera.
        o.targetFace?.let { face ->
            val t = size * 0.18f
            val g = size * 0.30f
            val (bar, glow) = when (face) {
                Facing.N -> RectF(rect.left, rect.top, rect.right, rect.top + t) to
                    RectF(rect.left, rect.top - g * 0.4f, rect.right, rect.top + g * 0.4f)
                Facing.S -> RectF(rect.left, rect.bottom - t, rect.right, rect.bottom) to
                    RectF(rect.left, rect.bottom - g * 0.4f, rect.right, rect.bottom + g * 0.4f)
                Facing.E -> RectF(rect.right - t, rect.top, rect.right, rect.bottom) to
                    RectF(rect.right - g * 0.4f, rect.top, rect.right + g * 0.4f, rect.bottom)
                Facing.W -> RectF(rect.left, rect.top, rect.left + t, rect.bottom) to
                    RectF(rect.left - g * 0.4f, rect.top, rect.left + g * 0.4f, rect.bottom)
            }
            canvas.drawRect(glow, faceGlowPaint)
            canvas.drawRect(bar, facePaint)
        }

        // C.5 wants the obstacle number small; C.9 wants the target ID large.
        val hasTarget = o.targetId != null
        val label = o.targetId?.toString() ?: o.id.toString()
        idPaint.color = Color.WHITE
        idPaint.textSize = if (hasTarget) size * 0.60f else size * 0.40f
        canvas.drawText(
            label, rect.centerX(), rect.centerY() + idPaint.textSize * 0.36f, idPaint,
        )

        // The character actually printed on the recognised image, as a corner
        // chip. The large number stays the ID, exactly as C.9 requires.
        // The character actually printed on the recognised image, on a chip that
        // hangs off the top-left corner. It has to overflow the cell to be
        // legible at all — a cell is only about 30dp on the real tablet — and
        // the dark ring is what stops it merging into the target face bar,
        // which is the same red.
        val glyph = o.targetId?.let { Arena.glyphFor(it) }
        if (glyph != null && size > dp(18f)) {
            // Sits clear of the block rather than on top of it: the large ID in
            // the middle is the checklist requirement and must stay readable.
            val chip = size * 0.58f
            val gx = rect.left - chip * 0.72f
            val gy = rect.top - chip * 0.72f
            rect2.set(gx, gy, gx + chip, gy + chip)
            canvas.drawRoundRect(rect2, chip * 0.30f, chip * 0.30f, glyphRingPaint)
            rect2.inset(chip * 0.09f, chip * 0.09f)
            canvas.drawRoundRect(rect2, chip * 0.24f, chip * 0.24f, glyphChipPaint)
            glyphPaint.textSize = chip * 0.70f
            canvas.drawText(
                glyph, rect2.centerX(), rect2.centerY() + glyphPaint.textSize * 0.35f, glyphPaint,
            )
        }
    }

    private fun drawRobot(canvas: Canvas) {
        val span = cell * Arena.ROBOT_SPAN
        val cx = cellCentreX2(poseX)
        val cy = cellCentreY2(poseY)

        canvas.save()
        canvas.rotate(poseBearing, cx, cy)

        // Soft halo so the robot separates from obstacles on camera.
        canvas.drawCircle(cx, cy, span * 0.56f, robotGlow)

        val half = span * 0.44f
        rect.set(cx - half, cy - half, cx + half, cy + half)
        canvas.drawRoundRect(rect, span * 0.16f, span * 0.16f, robotBody)
        canvas.drawRoundRect(rect, span * 0.16f, span * 0.16f, robotEdge)

        // Nose, drawn pointing up because the canvas is already rotated.
        val reach = span * 0.36f
        val wing = span * 0.24f
        path.reset()
        path.moveTo(cx, cy - reach)
        path.lineTo(cx - wing, cy + wing * 0.55f)
        path.lineTo(cx, cy + wing * 0.15f)
        path.lineTo(cx + wing, cy + wing * 0.55f)
        path.close()
        canvas.drawPath(path, nosePaint)

        canvas.restore()
    }

    private fun drawDragGhost(canvas: Canvas) {
        val id = dragId ?: return
        if (!dragging) return
        val o = state.obstacle(id) ?: return
        val outside = !inArena(dragX, dragY)

        if (outside) {
            rect.set(dragX - cell * 0.6f, dragY - cell * 0.6f, dragX + cell * 0.6f, dragY + cell * 0.6f)
            canvas.drawRoundRect(rect, cell * 0.2f, cell * 0.2f, dangerPaint)
            idPaint.color = Color.WHITE
            idPaint.textSize = cell * 0.5f
            canvas.drawText("×", rect.centerX(), rect.centerY() + idPaint.textSize * 0.36f, idPaint)
            return
        }

        val gx = cellXAt(dragX)
        val gy = cellYAt(dragY)
        if (gx in 0 until Arena.SIZE && gy in 0 until Arena.SIZE) {
            rect.set(cellLeft(gx), cellTop(gy), cellLeft(gx) + cell, cellTop(gy) + cell)
            canvas.drawRoundRect(rect, cell * 0.14f, cell * 0.14f, highlightPaint)
        }
        drawObstacle(canvas, o, dragX - cell * 0.66f, dragY - cell * 0.66f, cell * 1.28f)
    }

    /**
     * C.7. An obstacle is one cell out of twenty — about a fingertip wide — so
     * its four edges cannot be hit reliably. The checklist allows another
     * touch-based method, and this is it: tap the block and a magnified compass
     * opens over it.
     */
    private fun drawFaceSelector(canvas: Canvas) {
        val id = selectorFor ?: return
        val o = state.obstacle(id) ?: return

        canvas.drawRect(gridLeft, gridTop, gridLeft + cell * Arena.SIZE, gridBottom, scrimPaint)

        val cx = selectorCentreX(o)
        val cy = selectorCentreY(o)
        val outer = selectorRadius()
        val inner = outer * 0.34f

        Facing.entries.forEach { face ->
            path.reset()
            val sweepStart = when (face) {
                Facing.N -> 225f; Facing.E -> 315f; Facing.S -> 45f; Facing.W -> 135f
            }
            rect.set(cx - outer, cy - outer, cx + outer, cy + outer)
            path.arcTo(rect, sweepStart, 90f, true)
            rect.set(cx - inner, cy - inner, cx + inner, cy + inner)
            path.arcTo(rect, sweepStart + 90f, -90f, false)
            path.close()
            canvas.drawPath(path, if (o.targetFace == face) facePaint else quadrantPaint)
            canvas.drawPath(path, quadrantEdge)

            val mid = outer * 0.68f
            val lx = when (face) { Facing.E -> cx + mid; Facing.W -> cx - mid; else -> cx }
            val ly = when (face) { Facing.N -> cy - mid; Facing.S -> cy + mid; else -> cy }
            canvas.drawText(face.name, lx, ly + quadrantLabel.textSize * 0.35f, quadrantLabel)
        }

        drawObstacle(canvas, o, cx - inner * 0.7f, cy - inner * 0.7f, inner * 1.4f)
    }

    // -----------------------------------------------------------------
    // Touch — unchanged behaviour, this is what C.6 and C.7 were signed on
    // -----------------------------------------------------------------

    override fun onTouchEvent(event: MotionEvent): Boolean {
        if (cell <= 0f) return false
        val ex = event.x
        val ey = event.y

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                downX = ex; downY = ey; dragX = ex; dragY = ey
                dragging = false
                if (selectorFor != null) return true
                val gx = cellXAt(ex); val gy = cellYAt(ey)
                dragId = if (inArena(ex, ey)) state.obstacleAt(gx, gy)?.id else null
                if (inArena(ex, ey)) onHover(gx, gy)
                return true
            }

            MotionEvent.ACTION_MOVE -> {
                dragX = ex; dragY = ey
                if (selectorFor == null) {
                    if (!dragging && dragId != null && hypot(ex - downX, ey - downY) > slop) {
                        dragging = true
                    }
                    if (inArena(ex, ey)) onHover(cellXAt(ex), cellYAt(ey)) else onHoverEnd()
                }
                invalidate()
                return true
            }

            MotionEvent.ACTION_UP -> {
                onHoverEnd()
                val open = selectorFor
                if (open != null) {
                    resolveSelector(open, ex, ey)
                    selectorFor = null
                    dragId = null; dragging = false
                    invalidate()
                    return true
                }
                val id = dragId
                if (dragging && id != null) {
                    if (inArena(ex, ey)) onMoveObstacle(id, cellXAt(ex), cellYAt(ey))
                    else onRemoveObstacle(id)
                } else if (!dragging && inArena(ex, ey)) {
                    val gx = cellXAt(ex); val gy = cellYAt(ey)
                    val hit = state.obstacleAt(gx, gy)
                    if (hit != null) { selectorFor = hit.id; performClick() } else onAddObstacle(gx, gy)
                }
                dragId = null; dragging = false
                invalidate()
                return true
            }

            MotionEvent.ACTION_CANCEL -> {
                onHoverEnd()
                dragId = null; dragging = false; selectorFor = null
                invalidate()
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    private fun resolveSelector(id: Int, ex: Float, ey: Float) {
        val o = state.obstacle(id) ?: return
        val cx = selectorCentreX(o)
        val cy = selectorCentreY(o)
        val outer = selectorRadius()
        val inner = outer * 0.34f
        val d = hypot(ex - cx, ey - cy)
        if (d < inner || d > outer) return

        val degrees = Math.toDegrees(atan2((cy - ey).toDouble(), (ex - cx).toDouble()))
        val face = when {
            degrees >= 45 && degrees < 135 -> Facing.N
            degrees >= -45 && degrees < 45 -> Facing.E
            degrees >= -135 && degrees < -45 -> Facing.S
            else -> Facing.W
        }
        onSetTargetFace(id, face)
    }

    // -----------------------------------------------------------------
    // Geometry helpers
    // -----------------------------------------------------------------

    private fun cellLeft(x: Int) = gridLeft + x * cell
    private fun cellTop(y: Int) = gridBottom - (y + 1) * cell
    private fun cellCentreX(x: Int) = cellLeft(x) + cell / 2f
    private fun cellCentreY(y: Int) = cellTop(y) + cell / 2f
    private fun cellCentreX2(x: Float) = gridLeft + (x + 0.5f) * cell
    private fun cellCentreY2(y: Float) = gridBottom - (y + 0.5f) * cell

    private fun cellXAt(px: Float): Int =
        ((px - gridLeft) / cell).toInt().let { if (px < gridLeft) -1 else it }

    private fun cellYAt(py: Float): Int =
        ((gridBottom - py) / cell).toInt().let { if (py > gridBottom) -1 else it }

    private fun inArena(px: Float, py: Float): Boolean =
        px >= gridLeft && px < gridLeft + cell * Arena.SIZE && py >= gridTop && py < gridBottom

    private fun selectorRadius(): Float = maxOf(cell * 2.6f, dp(58f))

    private fun selectorCentreX(o: Obstacle): Float {
        val r = selectorRadius()
        return cellCentreX(o.x).coerceIn(gridLeft + r, gridLeft + cell * Arena.SIZE - r)
    }

    private fun selectorCentreY(o: Obstacle): Float {
        val r = selectorRadius()
        return cellCentreY(o.y).coerceIn(gridTop + r, gridBottom - r)
    }

    private fun withAlpha(color: Int, alpha: Int) =
        (color and 0x00FFFFFF) or (alpha.coerceIn(0, 255) shl 24)

    private companion object {
        /** Turn radii measured 31-Aug-2026, stm32/STM32_motion_spec.md. */
        const val RADIUS_FL_MM = 317f
        const val RADIUS_FR_MM = 413f
        const val LEAD_LEFT = 0.48f

        const val BOARD = 0xFF080D11.toInt()
        const val VIGNETTE = 0x3A000000
        const val GRID = 0xFF1B2831.toInt()
        const val GRID_MAJOR = 0xFF31454F.toInt()
        const val FRAME = 0xFF2E4150.toInt()
        const val START_FILL = 0x26F0A82E
        const val START_HATCH = 0x3DF0A82E
        const val AXIS = 0xFF6E7F8B.toInt()

        const val BLOCK_SHADOW = 0x55000000
        const val BLOCK_SIDE = 0xFF2B3841.toInt()
        const val BLOCK_TOP = 0xFF44555F.toInt()
        const val BLOCK_EDGE = 0xFF7B8F9E.toInt()

        const val TARGET = 0xFFFF5A3C.toInt()
        const val TARGET_GLOW = 0x44FF5A3C
        const val BOARD_RING = 0xFF080D11.toInt()

        const val ROBOT_BODY = 0xFF2489B8.toInt()
        const val ROBOT_EDGE = 0xFF7FD4FF.toInt()
        const val ROBOT_GLOW = 0x223FA8DE
        const val ROBOT_NOSE = 0xFFEAF8FF.toInt()
        const val TRAIL = 0xFF3FA8DE.toInt()

        const val ACCENT = 0xFFF0A82E.toInt()
        const val DANGER = 0xCCB0402A.toInt()
        const val SCRIM = 0xC0060A0D.toInt()
        const val QUADRANT = 0xF01B242D.toInt()
        const val QUADRANT_EDGE = 0xFF44555F.toInt()
    }
}
