"""Quarter-car active suspension schematic, drawn with Manim.

Mirrors the MATLAB diagram appended at the end of ``design_mpc.m``: the body
mass ``m_b`` and wheel mass ``m_w`` connected by the passive spring ``k_s``
and damper ``b_s`` in parallel with the active force actuator ``f_s``,
sitting on the tyre spring ``k_t`` and the road profile ``r``. A stylised
car silhouette is drawn behind the schematic to give it context, in the
spirit of the second reference image.

Render a single PNG with::

    pip install manim
    manim -sqh quarter_car_diagram.py QuarterCarSuspension

The ``-s`` flag tells Manim to save just the last frame as an image instead
of rendering a video. Drop ``-q h`` (or use ``-ql``) for a faster, lower
resolution image while iterating.
"""

from __future__ import annotations

import numpy as np
from manim import (
    Arc,
    Arrow,
    BLACK,
    Circle,
    DashedVMobject,
    Line,
    MathTex,
    PI,
    Polygon,
    Rectangle,
    RIGHT,
    Scene,
    Text,
    UP,
    VGroup,
    VMobject,
    WHITE,
)


CAR_BLUE = "#3D7FE6"


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
        np.array([xc - 3.4, mb_y_top + 0.05, 0.0]),
        np.array([xc - 2.7, mb_y_top + 0.05, 0.0]),
        np.array([xc - 1.6, mb_y_top + 0.95, 0.0]),
        np.array([xc - 0.3, mb_y_top + 1.50, 0.0]),
        np.array([xc + 1.6, mb_y_top + 1.50, 0.0]),
        np.array([xc + 2.4, mb_y_top + 1.00, 0.0]),
        np.array([xc + 3.0, mb_y_top + 0.30, 0.0]),
        np.array([xc + 3.4, mb_y_top + 0.05, 0.0]),
        np.array([xc + 3.4, mw_yc - 0.30, 0.0]),
        np.array([xc - 3.4, mw_yc - 0.30, 0.0]),
    ]
    body = Polygon(
        *pts,
        color=CAR_BLUE,
        stroke_width=2.5,
        fill_color=CAR_BLUE,
        fill_opacity=0.12,
    )

    # window glass hint (top edge + B-pillar)
    win_top = Line(
        np.array([xc - 1.4, mb_y_top + 1.40, 0.0]),
        np.array([xc + 1.4, mb_y_top + 1.40, 0.0]),
        color=CAR_BLUE, stroke_width=2,
    )
    win_div = Line(
        np.array([xc - 0.05, mb_y_top + 0.95, 0.0]),
        np.array([xc - 0.05, mb_y_top + 1.45, 0.0]),
        color=CAR_BLUE, stroke_width=2,
    )

    # wheel arch (upper half) + dashed tyre hint
    arch = Arc(
        radius=1.30, start_angle=0.0, angle=PI,
        arc_center=np.array([xc, mw_yc, 0.0]),
        color=CAR_BLUE, stroke_width=2.5,
    )
    tyre_circle = Circle(radius=1.05, color=CAR_BLUE, stroke_width=2)
    tyre_circle.move_to(np.array([xc, mw_yc, 0.0]))
    tyre = DashedVMobject(tyre_circle, num_dashes=32)

    return VGroup(body, win_top, win_div, arch, tyre)


# --- Scene -----------------------------------------------------------------


class QuarterCarSuspension(Scene):
    """Static schematic of the quarter-car active-suspension model."""

    def construct(self):
        # Layout (Manim units; frame is ~14.22 x 8)
        xc = 0.0
        mb_w, mb_h = 4.0, 0.7
        mw_w, mw_h = 4.0, 0.55
        mb_y_bot = 1.0                         # bottom of body mass
        mw_y_top = -0.7                        # top of wheel mass
        mw_y_bot = mw_y_top - mw_h
        road_y = -3.0

        spring_x = xc - mb_w / 2.0 + 0.55
        damp_x = xc
        act_x = xc + mb_w / 2.0 - 0.55

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
