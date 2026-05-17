"""Quarter-car active suspension schematic, drawn with Manim.

The body mass ``m_b`` and wheel mass ``m_w`` connected by the passive spring ``k_s``
and damper ``b_s`` in parallel with the active force actuator ``f_s``,
sitting on the tyre spring ``k_t`` and the road profile ``r``. A stylised
car silhouette is drawn behind the schematic to give it context.

Two scenes are provided:

* ``QuarterCarSuspension`` — the static schematic, rendered as a single PNG::

      pip install manim
      manim -sqh quarter_car_diagram.py QuarterCarSuspension

  The ``-s`` flag tells Manim to save just the last frame as an image
  instead of rendering a video. Drop ``-q h`` (or use ``-ql``) for a
  faster, lower-resolution image while iterating.

* ``QuarterCarSuspensionAnimation`` — the same schematic, animated from a
  simulation trace of ``(t, x_b, x_w, r)`` exported from the MATLAB /
  Simulink model. Render to MP4 with::

      manim -qh quarter_car_diagram.py QuarterCarSuspensionAnimation

  The default trace path is ``simulation.csv`` next to this script. Export
  it from MATLAB after running ``design_mpc`` with, e.g.::

      writematrix([t, xp(:,1), xp(:,3), r_signal], 'simulation.csv')

  (columns: time [s], body travel [m], wheel travel [m], road profile [m]).
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
from manim import (
    Arc,
    Arrow,
    BLACK,
    Circle,
    DashedVMobject,
    DecimalNumber,
    LEFT,
    Line,
    MathTex,
    PI,
    Polygon,
    Rectangle,
    RIGHT,
    Scene,
    Text,
    UP,
    UR,
    ValueTracker,
    VGroup,
    VMobject,
    WHITE,
    always_redraw,
    linear,
)


CAR_BLUE = "#3D7FE6"

DEFAULT_DATA_PATH = "simulation.csv"
# Real-world body/wheel/road motion is centimetre-scale, so we amplify it
# before feeding it into the schematic to keep the animation legible.
DEFAULT_DISPLAY_SCALE = 20.0


# --- Symbol builders -------------------------------------------------------


def make_spring(start, end, n_coils: int = 6, width: float = 0.20) -> VMobject:
    """Zig-zag spring between two 3-D points with ``n_coils`` zig-zags."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    L = float(np.linalg.norm(delta))
    if L == 0.0:
        return VMobject()

    unit = delta / L
    perp = np.array([-unit[1], unit[0], 0.0])
    lead = 0.12 * L                              # straight section at each end
    n_seg = 2 * n_coils

    pts = [start]
    for i in range(n_seg):
        t = lead + (L - 2.0 * lead) * i / (n_seg - 1)
        side = 1.0 if i % 2 == 0 else -1.0
        pts.append(start + unit * t + perp * width * side)
    pts.append(end)

    spring = VMobject(stroke_color=WHITE, stroke_width=3)
    spring.set_points_as_corners(pts)
    return spring


def make_damper(start, end, width: float = 0.18) -> VGroup:
    """Vertical dashpot symbol: open cup attached at ``start``, piston at ``end``."""
    x = float(start[0])
    y1 = float(start[1])
    y2 = float(end[1])
    L = y2 - y1

    cup = VMobject(stroke_color=WHITE, stroke_width=3)
    cup.set_points_as_corners([
        np.array([x - width, y1 + 0.55 * L, 0.0]),
        np.array([x - width, y1, 0.0]),
        np.array([x + width, y1, 0.0]),
        np.array([x + width, y1 + 0.55 * L, 0.0]),
    ])

    rod = Line(
        np.array([x, y2, 0.0]),
        np.array([x, y1 + 0.30 * L, 0.0]),
        color=WHITE, stroke_width=3,
    )
    piston = Line(
        np.array([x - 0.85 * width, y1 + 0.30 * L, 0.0]),
        np.array([x + 0.85 * width, y1 + 0.30 * L, 0.0]),
        color=WHITE, stroke_width=4,
    )
    return VGroup(cup, rod, piston)


def make_actuator(start, end, r: float = 0.25) -> VGroup:
    """Circle-with-diagonal-arrow symbol for a controllable force actuator."""
    x = float(start[0])
    y1 = float(start[1])
    y2 = float(end[1])
    yc = 0.5 * (y1 + y2)

    line_bot = Line(
        np.array([x, y1, 0.0]),
        np.array([x, yc - r, 0.0]),
        color=WHITE, stroke_width=3,
    )
    line_top = Line(
        np.array([x, yc + r, 0.0]),
        np.array([x, y2, 0.0]),
        color=WHITE, stroke_width=3,
    )
    circle = Circle(radius=r, color=WHITE, stroke_width=3).move_to(
        np.array([x, yc, 0.0])
    )
    a = 0.65 * r
    arrow = Arrow(
        start=np.array([x - a, yc - a, 0.0]),
        end=np.array([x + a, yc + a, 0.0]),
        buff=0.0,
        color=WHITE,
        stroke_width=3,
        max_tip_length_to_length_ratio=0.35,
    )
    return VGroup(line_bot, line_top, circle, arrow)


def make_road(center_x: float, y: float,
              width: float = 5.5, n_hatches: int = 15) -> VGroup:
    """Solid ground line with diagonal hatching beneath it."""
    line = Line(
        np.array([center_x - width / 2.0, y, 0.0]),
        np.array([center_x + width / 2.0, y, 0.0]),
        color=WHITE, stroke_width=4,
    )
    hatches = VGroup()
    xs = np.linspace(
        center_x - width / 2.0 + 0.15,
        center_x + width / 2.0 - 0.15,
        n_hatches,
    )
    for x in xs:
        hatches.add(Line(
            np.array([x, y, 0.0]),
            np.array([x - 0.22, y - 0.32, 0.0]),
            color=WHITE, stroke_width=2,
        ))
    return VGroup(line, hatches)


def make_car_silhouette(xc: float, mb_y_top: float,
                        mw_yc: float) -> VGroup:
    """Stylised side-profile car body, in pale blue, drawn behind the schematic."""
    pts = [
        np.array([xc - 3, mb_y_top + 1.5, 0.0]),
        np.array([xc + 2, mb_y_top + 1.5, 0.0]),
        np.array([xc + 4, mb_y_top + 0.7, 0.0]),
        np.array([xc + 4.5, mb_y_top + 0.6, 0.0]),
        np.array([xc + 4.5, mw_yc - 0.2, 0.0]),
        np.array([xc - 3, mw_yc - 0.2, 0.0]),
    ]
    body = Polygon(
        *pts,
        color=CAR_BLUE,
        stroke_width=2.5,
        fill_color=CAR_BLUE,
        fill_opacity=0.12,
    )

    # wheel arch (upper half) + dashed tyre hint
    arch = Arc(
        radius=1.9, start_angle=0.0, angle=PI,
        arc_center=np.array([xc, mw_yc, 0.0]),
        color=CAR_BLUE, stroke_width=2.5,
    )
    tyre_circle = Circle(radius=2, color=CAR_BLUE, stroke_width=5)
    tyre_circle.move_to(np.array([xc, mw_yc, 0.0]))
    tyre = DashedVMobject(tyre_circle, num_dashes=32)

    # No car silhouette for now
    return VGroup(tyre)


# --- Shared layout & simulation IO ----------------------------------------


def _layout() -> SimpleNamespace:
    """Layout constants (Manim units; frame is ~14.22 x 8).

    Shared between the static and animated scenes so both stay in sync.
    """
    L = SimpleNamespace()
    L.xc = 0.0
    L.mb_w, L.mb_h = 4.0, 0.7                  # body rectangle
    L.mw_w, L.mw_h = 4.0, 0.55                 # wheel rectangle
    L.mb_y_bot = 1.7                           # bottom of body mass
    L.mw_y_top = -0.7                          # top of wheel mass
    L.mw_y_bot = L.mw_y_top - L.mw_h
    L.road_y = -3.0
    L.spring_x = L.xc - L.mb_w / 2.0 + 0.55
    L.damp_x = L.xc
    L.act_x = L.xc + L.mb_w / 2.0 - 0.55
    return L


def _load_signal_csv(path: str):
    """Load a CSV of ``(t, x_b, x_w, r)`` in SI units (seconds, metres).

    The first row may be a header (e.g. ``t,x_b,x_w,r``) or numeric data.
    Returns four 1-D ``numpy`` arrays of the same length.
    """
    p = Path(path)
    if not p.is_absolute():
        p = Path(__file__).parent / p
    if not p.is_file():
        raise FileNotFoundError(
            f"Simulation trace not found at {p}. Export it from MATLAB after "
            "running design_mpc with, e.g.: "
            "writematrix([t, xp(:,1), xp(:,3), r_signal], 'simulation.csv')"
        )
    try:
        data = np.loadtxt(str(p), delimiter=",")
    except ValueError:
        # First row is a header.
        data = np.loadtxt(str(p), delimiter=",", skiprows=1)
    if data.ndim != 2 or data.shape[1] < 4:
        raise ValueError(
            f"Expected a 2-D CSV with at least 4 columns (t, x_b, x_w, r); "
            f"got shape {data.shape}."
        )
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3]


# --- Scenes ----------------------------------------------------------------


class QuarterCarSuspension(Scene):
    """Static schematic of the quarter-car active-suspension model."""

    def construct(self):
        L = _layout()
        xc = L.xc
        mb_w, mb_h = L.mb_w, L.mb_h
        mw_w, mw_h = L.mw_w, L.mw_h
        mb_y_bot = L.mb_y_bot
        mw_y_top = L.mw_y_top
        mw_y_bot = L.mw_y_bot
        road_y = L.road_y
        spring_x = L.spring_x
        damp_x = L.damp_x
        act_x = L.act_x

        # Body & wheel rectangles (opaque so they sit cleanly on the silhouette)
        body_rect = Rectangle(
            width=mb_w, height=mb_h,
            color=WHITE, stroke_width=3,
            fill_color=BLACK, fill_opacity=1.0,
        ).move_to(np.array([xc, mb_y_bot + mb_h / 2.0, 0.0]))

        wheel_rect = Rectangle(
            width=mw_w, height=mw_h,
            color=WHITE, stroke_width=3,
            fill_color=BLACK, fill_opacity=1.0,
        ).move_to(np.array([xc, mw_y_top - mw_h / 2.0, 0.0]))

        mb_label = MathTex("m_b").scale(1.0).move_to(body_rect)
        mw_label = MathTex("m_w").scale(1.0).move_to(wheel_rect)

        # Suspension stack between wheel top and body bottom
        spring_ks = make_spring(
            [spring_x, mw_y_top, 0.0],
            [spring_x, mb_y_bot, 0.0],
            n_coils=8, width=0.20,
        )
        damper_bs = make_damper(
            [damp_x, mw_y_top, 0.0],
            [damp_x, mb_y_bot, 0.0],
            width=0.18,
        )
        actuator_fs = make_actuator(
            [act_x, mw_y_top, 0.0],
            [act_x, mb_y_bot, 0.0],
            r=0.25,
        )
        ks_label = MathTex("k_s").scale(0.9).next_to(spring_ks, RIGHT, buff=0.1)
        bs_label = MathTex("b_s").scale(0.9).next_to(damper_bs, RIGHT, buff=0.1)
        fs_label = MathTex("f_s").scale(0.9).next_to(actuator_fs, RIGHT, buff=0.1)

        # Tyre spring + road
        spring_kt = make_spring(
            [xc, road_y, 0.0],
            [xc, mw_y_bot, 0.0],
            n_coils=6, width=0.20,
        )
        kt_label = MathTex("k_t").scale(0.9).next_to(spring_kt, RIGHT, buff=0.15)
        road = make_road(xc, road_y, width=5.5, n_hatches=15)

        # Vertical displacement arrows (pointing up) and disturbance arrow
        body_y_c = body_rect.get_center()[1]
        wheel_y_c = wheel_rect.get_center()[1]

        xb_arrow = Arrow(
            start=np.array([xc + mb_w / 2.0 + 0.55, body_y_c, 0.0]),
            end=np.array([xc + mb_w / 2.0 + 0.55, body_y_c + 0.9, 0.0]),
            buff=0.0, color=WHITE, stroke_width=3,
            max_tip_length_to_length_ratio=0.25,
        )
        xb_label = MathTex("x_b").scale(0.9).next_to(xb_arrow, RIGHT, buff=0.1)

        xw_arrow = Arrow(
            start=np.array([xc + mw_w / 2.0 + 0.55, wheel_y_c, 0.0]),
            end=np.array([xc + mw_w / 2.0 + 0.55, wheel_y_c + 0.9, 0.0]),
            buff=0.0, color=WHITE, stroke_width=3,
            max_tip_length_to_length_ratio=0.25,
        )
        xw_label = MathTex("x_w").scale(0.9).next_to(xw_arrow, RIGHT, buff=0.1)

        r_arrow = Arrow(
            start=np.array([xc + 3.0, road_y, 0.0]),
            end=np.array([xc + 3.0, road_y + 0.9, 0.0]),
            buff=0.0, color=WHITE, stroke_width=3,
            max_tip_length_to_length_ratio=0.25,
        )
        r_label = MathTex("r").scale(0.9).next_to(r_arrow, RIGHT, buff=0.1)

        # Stylised car body behind the schematic
        car = make_car_silhouette(
            xc=xc,
            mb_y_top=mb_y_bot + mb_h,
            mw_yc=wheel_y_c,
        )

        title = Text(
            "Quarter-car active suspension model",
            font_size=28,
        ).to_edge(UP, buff=0.25)

        # Build the static frame. Order matters for z-stacking: car silhouette
        # first, then the schematic on top, then labels.
        self.add(
            title,
            car,
            body_rect, wheel_rect,
            spring_ks, damper_bs, actuator_fs,
            spring_kt, road,
            xb_arrow, xw_arrow, r_arrow,
            mb_label, mw_label,
            ks_label, bs_label, fs_label,
            kt_label,
            xb_label, xw_label, r_label,
        )


class QuarterCarSuspensionAnimation(Scene):
    """Animated quarter-car driven by a simulation trace.

    Reads a CSV with columns ``(t, x_b, x_w, r)`` in SI units and animates
    the body mass, wheel mass, road surface, springs, damper and actuator
    in real time. The static schematic class above is left untouched, so the
    same script renders either a still PNG or an MP4 depending on which
    Scene is selected on the command line.

    Override via subclassing (or by editing the class attributes) to point
    at a different CSV or change the amplification factor::

        manim -qh quarter_car_diagram.py QuarterCarSuspensionAnimation
    """

    # Path to a CSV with columns (t [s], x_b [m], x_w [m], r [m]).
    # Relative paths are resolved next to this script.
    DATA_PATH: str = DEFAULT_DATA_PATH

    # Visual amplification of the (cm-scale) physical displacements.
    DISPLAY_SCALE: float = DEFAULT_DISPLAY_SCALE

    # Playback speed: 1.0 = real time, 0.5 = half-speed (slow-mo), etc.
    PLAYBACK_SPEED: float = 1.0

    def construct(self):
        L = _layout()
        t_data, xb_data, xw_data, r_data = _load_signal_csv(self.DATA_PATH)
        s = float(self.DISPLAY_SCALE)
        T = float(t_data[-1])

        # Baseline (zero-displacement) positions
        body_y_c0 = L.mb_y_bot + L.mb_h / 2.0
        wheel_y_c0 = L.mw_y_top - L.mw_h / 2.0

        # ValueTracker holds the current simulation time; updaters read from it
        time_tracker = ValueTracker(0.0)

        def get_xb() -> float:
            return float(np.interp(time_tracker.get_value(), t_data, xb_data)) * s

        def get_xw() -> float:
            return float(np.interp(time_tracker.get_value(), t_data, xw_data)) * s

        def get_r() -> float:
            return float(np.interp(time_tracker.get_value(), t_data, r_data)) * s

        # --- Bodies that translate vertically ---
        body_rect = Rectangle(
            width=L.mb_w, height=L.mb_h,
            color=WHITE, stroke_width=3,
            fill_color=BLACK, fill_opacity=1.0,
        ).move_to(np.array([L.xc, body_y_c0, 0.0]))
        body_rect.add_updater(
            lambda m: m.move_to(np.array([L.xc, body_y_c0 + get_xb(), 0.0]))
        )

        mb_label = MathTex("m_b").scale(1.0).move_to(body_rect)
        mb_label.add_updater(lambda m: m.move_to(body_rect))

        wheel_rect = Rectangle(
            width=L.mw_w, height=L.mw_h,
            color=WHITE, stroke_width=3,
            fill_color=BLACK, fill_opacity=1.0,
        ).move_to(np.array([L.xc, wheel_y_c0, 0.0]))
        wheel_rect.add_updater(
            lambda m: m.move_to(np.array([L.xc, wheel_y_c0 + get_xw(), 0.0]))
        )

        mw_label = MathTex("m_w").scale(1.0).move_to(wheel_rect)
        mw_label.add_updater(lambda m: m.move_to(wheel_rect))

        # Tyre silhouette: dashed circle that rides with the wheel
        tyre_circle = Circle(radius=2, color=CAR_BLUE, stroke_width=5)
        tyre_circle.move_to(np.array([L.xc, wheel_y_c0, 0.0]))
        tyre = DashedVMobject(tyre_circle, num_dashes=32)
        tyre.add_updater(
            lambda m: m.move_to(np.array([L.xc, wheel_y_c0 + get_xw(), 0.0]))
        )

        # --- Springs / damper / actuator are rebuilt every frame because
        # their internal geometry (zig-zag corners, piston lines) depends on
        # the live endpoints rather than a simple affine transform.
        spring_ks = always_redraw(lambda: make_spring(
            [L.spring_x, L.mw_y_top + get_xw(), 0.0],
            [L.spring_x, L.mb_y_bot + get_xb(), 0.0],
            n_coils=8, width=0.20,
        ))
        damper_bs = always_redraw(lambda: make_damper(
            [L.damp_x, L.mw_y_top + get_xw(), 0.0],
            [L.damp_x, L.mb_y_bot + get_xb(), 0.0],
            width=0.18,
        ))
        actuator_fs = always_redraw(lambda: make_actuator(
            [L.act_x, L.mw_y_top + get_xw(), 0.0],
            [L.act_x, L.mb_y_bot + get_xb(), 0.0],
            r=0.25,
        ))
        spring_kt = always_redraw(lambda: make_spring(
            [L.xc, L.road_y + get_r(), 0.0],
            [L.xc, L.mw_y_bot + get_xw(), 0.0],
            n_coils=6, width=0.20,
        ))

        # Road (rebuilt every frame so the hatching stays attached to the line)
        road = always_redraw(lambda: make_road(
            L.xc, L.road_y + get_r(),
            width=5.5, n_hatches=15,
        ))

        # Labels next to the live elements
        ks_label = MathTex("k_s").scale(0.9)
        ks_label.add_updater(lambda m: m.next_to(spring_ks, RIGHT, buff=0.1))
        bs_label = MathTex("b_s").scale(0.9)
        bs_label.add_updater(lambda m: m.next_to(damper_bs, RIGHT, buff=0.1))
        fs_label = MathTex("f_s").scale(0.9)
        fs_label.add_updater(lambda m: m.next_to(actuator_fs, RIGHT, buff=0.1))
        kt_label = MathTex("k_t").scale(0.9)
        kt_label.add_updater(lambda m: m.next_to(spring_kt, RIGHT, buff=0.15))

        # Title and an HUD showing the current simulation time.
        title = Text(
            "Quarter-car active suspension - simulation",
            font_size=28,
        ).to_edge(UP, buff=0.25)

        time_caption = MathTex("t = ", font_size=32)
        time_unit = MathTex(r"\,\mathrm{s}", font_size=32)
        time_value = DecimalNumber(0.0, num_decimal_places=2, font_size=32)
        time_value.add_updater(lambda m: m.set_value(time_tracker.get_value()))
        time_hud = VGroup(time_caption, time_value, time_unit).arrange(
            RIGHT, buff=0.08
        ).to_corner(UR, buff=0.3)

        # Z-stack: tyre first, then road, then schematic on top.
        self.add(
            title,
            tyre,
            road,
            body_rect, wheel_rect,
            spring_ks, damper_bs, actuator_fs,
            spring_kt,
            mb_label, mw_label,
            ks_label, bs_label, fs_label, kt_label,
            time_hud,
        )

        # Play the trace. Manim integrates the ValueTracker linearly over
        # run_time, so the on-screen time matches the simulation time
        # (scaled by PLAYBACK_SPEED).
        run_time = max(T / float(self.PLAYBACK_SPEED), 1e-3)
        self.play(
            time_tracker.animate.set_value(T),
            run_time=run_time,
            rate_func=linear,
        )
