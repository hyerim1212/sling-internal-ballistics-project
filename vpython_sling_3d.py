from math import cos, pi, sin

from sling_model.sling_model import (
    SlingParams,
    angular_acceleration,
    drag_range,
    drag_speed_profile,
    max_launch_angle,
    release_speed,
    spin_rate,
    timing_error,
    vacuum_range,
    windup_time,
)

from vpython import*

params = SlingParams()
running = True
show_external = True
time_s = 0.0
theta = -2 * pi * params.rotations
plane_tilt_deg = 48.0

scene = canvas(
    title="3D pirouette-style sling with flight trajectory",
    width=1120,
    height=720,
    background=vector(128, 128, 128),
    align="left",
)
scene.forward = vector(-0.7, -0.35, -1)
scene.center = vector(0.8, 0.4, 0)
scene.range = 5.2

speed_graph = graph(
    title="Release speed v - power P",
    xtitle="Power P (W)",
    ytitle="Release speed v (m/s)",
    width=440,
    height=260,
    background=color.white,
    align="left"
    
)
speed_curve = None
speed_point = None
position_graph = graph(
    title="External speed v - position x",
    xtitle="Horizontal position x (m)",
    ytitle="Speed v (m/s)",
    width=440,
    height=260,
    background=color.white,
    align="left"
)
position_curve = None

scene.append_to_caption("<div style='clear: both;'></div>\n")

origin = sphere(pos=vector(0, 0, 0), radius=0.07, color=color.black)
bullet = sphere(pos=vector(0, 0, 0), radius=0.13, color=color.cyan, make_trail=True)
cord = curve(pos=[origin.pos, bullet.pos], color=color.gray(0.45), radius=0.014)
orbit = curve(color=color.gray(0.75), radius=0.006)
sling_trail = curve(color=color.gray(0.5), radius=0.018)
trajectory_path = curve(color=color.blue, radius=0.018)
velocity_arrow = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 0), color=color.red, shaftwidth=0.055)
plane_marker = ring(pos=origin.pos, axis=vector(0, 0, 1), radius=params.radius_m, thickness=0.012, color=color.gray(0.82))

def on_slider_change(_event=None) -> None:
    return None


power_text = wtext(text="")
power_slider = slider(min=50, max=600, value=params.power_w, step=10, bind=on_slider_change)
scene.append_to_caption("\n")
mass_text = wtext(text="")
mass_slider = slider(min=0.03, max=1.2, value=params.bullet_mass_kg, step=0.01, bind=on_slider_change)
scene.append_to_caption("\n")
radius_text = wtext(text="")
radius_slider = slider(min=0.8, max=3.5, value=params.radius_m, step=0.05, bind=on_slider_change)
scene.append_to_caption("\n")
rotations_text = wtext(text="")
rotations_slider = slider(min=1, max=6, value=params.rotations, step=0.5, bind=on_slider_change)
scene.append_to_caption("\n")
height_text = wtext(text="")
height_slider = slider(min=0.8, max=2.1, value=params.shoulder_height_m, step=0.05, bind=on_slider_change)
scene.append_to_caption("\n")
bullet_radius_text = wtext(text="")
bullet_radius_slider = slider(min=0.004, max=0.05, value=params.bullet_radius_m, step=0.001, bind=on_slider_change)
scene.append_to_caption("\n")
tilt_text = wtext(text="")
tilt_slider = slider(min=0, max=70, value=plane_tilt_deg, step=1, bind=on_slider_change)
scene.append_to_caption("\n\n")


def toggle_running(_event=None) -> None:
    global running
    running = not running


def toggle_external(_event=None) -> None:
    global show_external
    show_external = not show_external
    reset_motion()


def reset_motion(_event=None) -> None:
    global time_s, theta
    p = current_params()
    time_s = 0.0
    theta = -2 * pi * p.rotations
    bullet.clear_trail()
    sling_trail.clear()
    trajectory_path.clear()
    velocity_arrow.axis = vector(0, 0, 0)


button(text="Pause / Run", bind=toggle_running)
scene.append_to_caption("  ")
button(text="Reset", bind=reset_motion)
scene.append_to_caption("  ")
button(text="Flight trajectory on/off", bind=toggle_external)
scene.append_to_caption("\n\n")
info = wtext(text="")


def current_params() -> SlingParams:
    radius = max(radius_slider.value, height_slider.value + 0.01)
    return SlingParams(
        power_w=power_slider.value,
        rotations=rotations_slider.value,
        radius_m=radius,
        bullet_mass_kg=mass_slider.value,
        shoulder_height_m=height_slider.value,
        bullet_radius_m=bullet_radius_slider.value,
    )


def basis_vectors(p: SlingParams) -> tuple[vector, vector, vector, float, float]:
    gamma = max_launch_angle(p.shoulder_height_m, p.radius_m)
    release_angle = gamma - pi / 2
    tangent = vector(cos(gamma), sin(gamma), 0)
    radial_flat = vector(cos(release_angle), sin(release_angle), 0)
    normal_flat = vector(0, 0, 1)
    tilt = tilt_slider.value * pi / 180
    radial = radial_flat * cos(tilt) + normal_flat * sin(tilt)
    normal = radial.cross(tangent).norm()
    return radial.norm(), tangent.norm(), normal, gamma, release_angle


def set_orbit_points(p: SlingParams, radial: vector, tangent: vector) -> None:
    orbit.clear()
    for i in range(241):
        a = 2 * pi * i / 240
        orbit.append(pos=p.radius_m * (cos(a) * radial + sin(a) * tangent))


def update_speed_graph(p: SlingParams) -> None:
    global speed_curve, speed_point, position_curve
    if speed_curve is not None:
        speed_curve.delete()
    if speed_point is not None:
        speed_point.delete()
    if position_curve is not None:
        position_curve.delete()

    speed_curve = gcurve(graph=speed_graph, color=color.blue)
    for power in range(50, 601, 10):
        speed_curve.plot(power, release_speed(power, p.rotations, p.radius_m, p.bullet_mass_kg))

    current_v = release_speed(p.power_w, p.rotations, p.radius_m, p.bullet_mass_kg)
    speed_point = gdots(graph=speed_graph, color=color.red, size=10)
    speed_point.plot(p.power_w, current_v)

    gamma = max_launch_angle(p.shoulder_height_m, p.radius_m)
    position_curve = gcurve(graph=position_graph, color=color.green)
    profile = drag_speed_profile(
        current_v,
        gamma,
        p.bullet_mass_kg,
        p.bullet_radius_m,
        p.drag_coefficient,
        release_height_m=p.shoulder_height_m,
        dt=0.01,
    )
    step = max(1, len(profile) // 220)
    for x, speed in profile[::step]:
        position_curve.plot(x, speed)


def update_static(p: SlingParams) -> tuple[float, float, float, vector, vector, list[vector]]:
    radial, tangent, normal, gamma, _release_angle = basis_vectors(p)
    v = release_speed(p.power_w, p.rotations, p.radius_m, p.bullet_mass_kg)
    t_windup = windup_time(p.rotations, p.radius_m, v)
    range_drag, trajectory = drag_range(
        v,
        gamma,
        p.bullet_mass_kg,
        p.bullet_radius_m,
        p.drag_coefficient,
        release_height_m=p.shoulder_height_m,
        dt=0.004,
    )
    spin_hz = spin_rate(v, p.radius_m, p.bullet_radius_m)
    dt_ms = 1000 * timing_error(0.40, p.radius_m, 20.0, gamma)
    range_vacuum = vacuum_range(v, gamma)

    power_text.text = f"Power P: {p.power_w:5.0f} W"
    mass_text.text = f"Bullet mass m: {p.bullet_mass_kg:5.3f} kg"
    radius_text.text = f"Sling radius r: {p.radius_m:4.2f} m"
    rotations_text.text = f"Rotations n: {p.rotations:3.1f}"
    height_text.text = f"Shoulder height h: {p.shoulder_height_m:4.2f} m"
    bullet_radius_text.text = f"Bullet radius b: {p.bullet_radius_m:5.3f} m"
    tilt_text.text = f"Sling plane tilt: {tilt_slider.value:4.0f} deg"

    info.text = (
        "3D sling motion: tilted internal plane + post-release flight trajectory\n"
        f"v = {v:4.1f} m/s, wind-up = {t_windup:4.2f} s, "
        f"launch angle = {gamma * 180 / pi:4.1f} deg\n"
        f"range with drag = {range_drag:5.1f} m, vacuum range = {range_vacuum:5.1f} m, "
        f"spin = {spin_hz:5.1f} Hz, timing = +/- {dt_ms:4.2f} ms\n"
        f"Flight trajectory overlay: {'on' if show_external else 'off'}"
    ).replace("\n", "<br>")
    update_speed_graph(p)

    set_orbit_points(p, radial, tangent)
    plane_marker.radius = p.radius_m
    plane_marker.axis = normal
    bullet.radius = max(0.055, min(0.16, 2.5 * p.bullet_radius_m))

    release_pos = p.radius_m * radial
    scale = max(6.0, range_drag / 8.0)
    x0, y0 = trajectory[0]
    step = max(1, len(trajectory) // 260)
    flight_points = [release_pos + vector((x - x0) / scale, (y - y0) / scale, 0) for x, y in trajectory[::step]]
    return v, gamma, t_windup, radial, tangent, flight_points


last_signature = None
flight_points: list[vector] = []
flight_index = 0
phase = "windup"

while True:
    rate(150)
    p = current_params()
    signature = (
        p.power_w,
        p.rotations,
        p.radius_m,
        p.bullet_mass_kg,
        p.shoulder_height_m,
        p.bullet_radius_m,
        tilt_slider.value,
        show_external,
    )
    if signature != last_signature:
        reset_motion()
        v_release, gamma_release, windup, radial_dir, tangent_dir, flight_points = update_static(p)
        alpha = angular_acceleration(p.rotations, p.radius_m, v_release)
        flight_index = 0
        phase = "windup"
        last_signature = signature

    if running:
        if phase == "windup":
            time_s += 1 / 60
            theta = -2 * pi * p.rotations + 0.5 * alpha * time_s * time_s
            bullet.pos = p.radius_m * (cos(theta) * radial_dir + sin(theta) * tangent_dir)
            sling_trail.append(pos=bullet.pos)
            if theta >= 0:
                theta = 0
                phase = "flight" if show_external else "release"
                flight_index = 0
                bullet.clear_trail()
                trajectory_path.clear()
                bullet.pos = p.radius_m * radial_dir
                velocity_arrow.pos = bullet.pos
                velocity_arrow.axis = tangent_dir * min(1.8, v_release / 28)
        elif phase == "flight":
            if flight_index < len(flight_points):
                bullet.pos = flight_points[flight_index]
                trajectory_path.append(pos=bullet.pos)
                flight_index += 1
            else:
                reset_motion()
                phase = "windup"

    if phase == "windup":
        velocity_arrow.axis = vector(0, 0, 0)

    cord.modify(0, pos=origin.pos)
    cord.modify(1, pos=bullet.pos)
