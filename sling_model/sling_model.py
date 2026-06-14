"""Physics model for Mark Denny, "Internal ballistics of the sling".

The functions implement the pirouette-style sling approximations used in the
paper: constant angular acceleration during wind-up, massless stiff cords, no
gravity/drag during internal ballistics, and quadratic drag after release.
The public `summarize` entry point is style-aware so Section III models can be
added without changing the controller program.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, cos, pi, sin, sqrt


G = 9.81
RHO_AIR = 1.225


@dataclass(frozen=True)
class SlingParams:
    style: str = "section_ii_pirouette"
    power_w: float = 200.0
    rotations: float = 3.0
    radius_m: float = 2.5
    bullet_mass_kg: float = 0.454
    shoulder_height_m: float = 1.6
    bullet_radius_m: float = 0.026
    drag_coefficient: float = 0.4
    air_density: float = RHO_AIR
    gravity: float = G


STYLE_LABELS = {
    "section_ii_pirouette": "Section II: pirouette style",
    "section_iii_placeholder": "Section III: side-arm style placeholder",
}


def release_speed(power_w: float, rotations: float, radius_m: float, bullet_mass_kg: float) -> float:
    """Eq. (1): launch speed from average mechanical power."""
    return 2.0 * (pi * radius_m * rotations * power_w / bullet_mass_kg) ** (1.0 / 3.0)


def windup_time(rotations: float, radius_m: float, speed_m_s: float) -> float:
    """Wind-up duration implied by constant angular acceleration."""
    return 4.0 * pi * rotations * radius_m / speed_m_s


def angular_acceleration(rotations: float, radius_m: float, speed_m_s: float) -> float:
    """Constant angular acceleration alpha used during wind-up."""
    return speed_m_s * speed_m_s / (4.0 * pi * rotations * radius_m * radius_m)


def max_launch_angle(shoulder_height_m: float, radius_m: float) -> float:
    """Maximum release angle for the pirouette geometry in Fig. 2(a)."""
    ratio = max(-1.0, min(1.0, shoulder_height_m / radius_m))
    return asin(ratio)


def spin_rate(speed_m_s: float, radius_m: float, bullet_radius_m: float) -> float:
    """Eq. (2): approximate bullet spin frequency in Hz."""
    return speed_m_s / sqrt(2.0 * pi * radius_m * bullet_radius_m)


def timing_error(width_m: float, sling_radius_m: float, range_m: float, launch_angle_rad: float, gravity: float = G) -> float:
    """Eq. (3): maximum release timing error for a target of width W at range R."""
    return width_m * sling_radius_m * sqrt(sin(2.0 * launch_angle_rad) / (4.0 * gravity * range_m**3))


def vacuum_range(speed_m_s: float, launch_angle_rad: float, gravity: float = G) -> float:
    """Flat-ground projectile range without aerodynamic drag."""
    return speed_m_s * speed_m_s * sin(2.0 * launch_angle_rad) / gravity


def drag_range(
    speed_m_s: float,
    launch_angle_rad: float,
    mass_kg: float,
    bullet_radius_m: float,
    drag_coefficient: float = 0.4,
    air_density: float = RHO_AIR,
    release_height_m: float = 0.0,
    gravity: float = G,
    dt: float = 0.001,
    max_time_s: float = 60.0,
) -> tuple[float, list[tuple[float, float]]]:
    """Projectile trajectory and range with quadratic drag.

    Returns (range_m, trajectory), where trajectory is a list of (x, y) points.
    The bullet is treated as a sphere with frontal area pi*b^2, matching the
    simplified drag estimate used for the Section II range plot.
    """
    area = pi * bullet_radius_m * bullet_radius_m
    k = 0.5 * air_density * drag_coefficient * area / mass_kg
    x = 0.0
    y = release_height_m
    vx = speed_m_s * cos(launch_angle_rad)
    vy = speed_m_s * sin(launch_angle_rad)
    trajectory = [(x, y)]
    previous = (x, y)
    t = 0.0

    while t < max_time_s:
        previous = (x, y)
        speed = sqrt(vx * vx + vy * vy)
        ax = -k * speed * vx
        ay = -gravity - k * speed * vy
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        t += dt
        trajectory.append((x, y))
        if y <= 0.0 and t > dt:
            break

    if trajectory[-1][1] <= 0.0:
        x0, y0 = previous
        x1, y1 = trajectory[-1]
        fraction = y0 / (y0 - y1) if y0 != y1 else 0.0
        landing_x = x0 + fraction * (x1 - x0)
        trajectory[-1] = (landing_x, 0.0)
        return landing_x, trajectory

    return trajectory[-1][0], trajectory


def drag_speed_profile(
    speed_m_s: float,
    launch_angle_rad: float,
    mass_kg: float,
    bullet_radius_m: float,
    drag_coefficient: float = 0.4,
    air_density: float = RHO_AIR,
    release_height_m: float = 0.0,
    gravity: float = G,
    dt: float = 0.004,
    max_time_s: float = 60.0,
) -> list[tuple[float, float]]:
    """Return (horizontal position, speed) during external flight with drag."""
    area = pi * bullet_radius_m * bullet_radius_m
    k = 0.5 * air_density * drag_coefficient * area / mass_kg
    x = 0.0
    y = release_height_m
    vx = speed_m_s * cos(launch_angle_rad)
    vy = speed_m_s * sin(launch_angle_rad)
    profile = [(x, speed_m_s)]
    t = 0.0

    while t < max_time_s:
        speed = sqrt(vx * vx + vy * vy)
        ax = -k * speed * vx
        ay = -gravity - k * speed * vy
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        t += dt
        profile.append((x, sqrt(vx * vx + vy * vy)))
        if y <= 0.0 and t > dt:
            break

    return profile


def section_ii_pirouette_summary(params: SlingParams = SlingParams()) -> dict[str, float]:
    """Compute the main Section II pirouette-style quantities."""
    v = release_speed(params.power_w, params.rotations, params.radius_m, params.bullet_mass_kg)
    gamma = max_launch_angle(params.shoulder_height_m, params.radius_m)
    range_drag, _ = drag_range(
        v,
        gamma,
        params.bullet_mass_kg,
        params.bullet_radius_m,
        params.drag_coefficient,
        params.air_density,
        release_height_m=params.shoulder_height_m,
        gravity=params.gravity,
    )
    return {
        "release_speed_m_s": v,
        "windup_time_s": windup_time(params.rotations, params.radius_m, v),
        "launch_angle_deg": gamma * 180.0 / pi,
        "vacuum_range_m": vacuum_range(v, gamma, params.gravity),
        "drag_range_m": range_drag,
        "spin_rate_hz": spin_rate(v, params.radius_m, params.bullet_radius_m),
        "timing_error_20m_40cm_ms": 1000.0
        * timing_error(0.40, params.radius_m, 20.0, gamma, params.gravity),
    }


def section_iii_placeholder_summary(params: SlingParams = SlingParams()) -> dict[str, float]:
    """Reserved entry point for Section III.

    Section III uses a more complicated sling path than the one-plane pirouette
    model. Keeping this function explicit makes the intended extension point
    visible without pretending that Section III physics has been implemented.
    """
    raise NotImplementedError("Section III model has not been implemented yet.")


def summarize(params: SlingParams = SlingParams()) -> dict[str, float]:
    """Compute the main quantities for the selected sling style."""
    if params.style == "section_ii_pirouette":
        return section_ii_pirouette_summary(params)
    if params.style == "section_iii_placeholder":
        return section_iii_placeholder_summary(params)
    raise ValueError(f"Unknown sling style: {params.style}")


if __name__ == "__main__":
    for key, value in summarize().items():
        print(f"{key}: {value:.3f}")
