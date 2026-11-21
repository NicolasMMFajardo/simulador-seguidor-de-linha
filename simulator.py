"""Modelo do robô, telemetria e ferramentas offboard."""
from bisect import bisect_right
from dataclasses import dataclass
from math import atan2, ceil, cos, exp, hypot, isfinite, pi, sin, sqrt
from random import Random

DT = 0.01
LINE = 0.019
GRAVITY = 9.81
SENSOR_BOARD_BACK = .007
SENSOR_BOARD_FRONT = .004
CURVE_LEVELS = ("Reta", "5", "4", "3", "2", "1")
CURVE_CENTERS = (0, .65, 1.15, 1.9, 3.0, 4.6)


@dataclass(frozen=True)
class Robot:
    wheelbase: float = 0.164
    diameter: float = 0.032
    ticks_per_turn: int = 360
    wheel_limit: float | None = None
    motor_tau: float = 0.075
    lateral_limit: float = 3.0
    gyro_bias: float = 0.015
    motor_rpm: float = 1500
    sensor_count: int = 8
    sensor_board_width: float = 0.065
    sensor_span: float = 0.055
    body_length: float = 0.060
    body_width: float = 0.180
    body_height: float = 0.070
    acceleration_limit: float = 1.2
    braking_limit: float = .65
    straight_kp_speed: float | None = .85
    sensor_gap: float = .080
    axle_from_rear: float = .015
    wheel_width: float = .016
    mapping_speed: float = .27
    mapping_kp: float = 100
    mapping_kd: float = 6
    suction_enabled: bool = False
    suction_force_n: float | None = None
    mass_kg: float | None = None

    @property
    def meters_per_tick(self):
        return pi * self.diameter / self.ticks_per_turn

    @property
    def max_wheel_speed(self):
        if self.wheel_limit is not None:
            return self.wheel_limit
        return pi * self.diameter * self.motor_rpm / 60

    @property
    def effective_lateral_limit(self):
        if not self.suction_enabled:
            return self.lateral_limit
        if self.mass_kg is None or not isfinite(self.mass_kg) or self.mass_kg <= 0:
            raise ValueError("Informe a massa positiva do robô para usar a sucção.")
        if self.suction_force_n is None or not isfinite(self.suction_force_n) or self.suction_force_n <= 0:
            raise ValueError("Informe a força de sucção positiva, em newtons.")
        limit = self.lateral_limit * (1 + self.suction_force_n/(self.mass_kg*GRAVITY))
        if not isfinite(limit):
            raise ValueError("A massa e a força informadas geram um limite de aderência inválido.")
        return limit

    @property
    def sensor_offsets(self):
        return tuple(self.sensor_span * (i / (self.sensor_count - 1) - .5)
                     for i in range(self.sensor_count))

    @property
    def body_edges(self):
        return -self.axle_from_rear, self.body_length-self.axle_from_rear

    @property
    def body_center(self):
        return self.body_length/2-self.axle_from_rear

    @property
    def sensor_forward(self):
        return self.body_edges[1] + self.sensor_gap

    @property
    def sensor_board_edges(self):
        return self.sensor_forward-SENSOR_BOARD_BACK, self.sensor_forward+SENSOR_BOARD_FRONT

    @property
    def footprint_boxes(self):
        back, front = self.sensor_board_edges
        return ((self.body_center, self.body_length/2, self.body_width/2),
                (0, self.diameter/2, (self.wheelbase+self.wheel_width)/2),
                ((front+back)/2, (front-back)/2, self.sensor_board_width/2))

    @property
    def overall_length(self):
        boxes = self.footprint_boxes
        return max(x+half for x, half, _ in boxes)-min(x-half for x, half, _ in boxes)

    @property
    def overall_width(self):
        return max(self.body_width, self.wheelbase+self.wheel_width, self.sensor_board_width)


@dataclass(frozen=True)
class Frame:
    x: float
    y: float
    heading: float
    velocity: float = 0.0
    traction_long: float = 0.0
    traction_lat: float = 0.0
    readings: list[float] | None = None
    segment: int | None = None
    target_speed: float = 0.0
    progress_m: float = 0.0
    steering_kp: float | None = None
    planned_speed: float | None = None

    @property
    def grip_demand(self):
        return abs(self.traction_lat)


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def example():
    return [(1.30 + 1.02 * cos(a), 0.81 + (0.56 + 0.14 * cos(a)) * sin(a))
            for a in (i * 2 * pi / 80 for i in range(80))]


class Track:
    """Polilinha fechada em metros. O primeiro ponto marca a largada."""
    def __init__(self, points):
        self.points = [tuple(map(float, p)) for p in points]
        self.segments = []
        self.starts = []
        self.grid = {}
        self.length = 0.0
        for (x, y), (end_x, end_y) in zip(self.points, self.points[1:] + self.points[:1]):
            dx, dy = end_x - x, end_y - y
            length = hypot(dx, dy)
            if length < 1e-6:
                continue
            index = len(self.segments)
            self.starts.append(self.length)
            self.segments.append((x, y, dx, dy, length))
            self.length += length
            for gx in range(int(min(x, end_x) // .1) - 1, int(max(x, end_x) // .1) + 2):
                for gy in range(int(min(y, end_y) // .1) - 1, int(max(y, end_y) // .1) + 2):
                    self.grid.setdefault((gx, gy), []).append(index)
        if len(self.segments) < 3 or self.length < .4:
            raise ValueError("Desenhe uma pista fechada com pelo menos 0,40 m.")

    def pose(self, distance):
        distance %= self.length
        index = bisect_right(self.starts, distance) - 1
        x, y, dx, dy, length = self.segments[index]
        fraction = (distance-self.starts[index])/length
        return x+dx*fraction, y+dy*fraction, atan2(dy, dx)

    def nearest(self, x, y, *, around=None):
        best = float("inf")
        along = 0.0
        nearby = self.grid.get((int(x // .1), int(y // .1)))
        for i in nearby or range(len(self.segments)):
            sx, sy, dx, dy, length = self.segments[i]
            fraction = clamp(((x - sx) * dx + (y - sy) * dy) / length**2, 0, 1)
            position = self.starts[i] + length*fraction
            if around is not None and abs((position-around+self.length/2) % self.length-self.length/2) > .22:
                continue
            distance = (x - sx - dx * fraction)**2 + (y - sy - dy * fraction)**2
            if distance < best:
                best, along = distance, position
        if best == float("inf") and around is not None:
            return best, around % self.length
        return sqrt(best), along


def body_over_line(track, x, y, heading, half_length, half_width, *, forward=0.0):
    """Interseção do retângulo do robô com a faixa em torno da polilinha."""
    c, s = cos(heading), sin(heading)
    x, y = x+forward*c, y+forward*s
    radius = hypot(half_length, half_width) + LINE/2
    candidates = {index for i in range(int((x-radius)//.1), int((x+radius)//.1)+1)
                  for j in range(int((y-radius)//.1), int((y+radius)//.1)+1)
                  for index in track.grid.get((i,j), ())}
    for index in candidates:
        tx, ty, dx, dy, length = track.segments[index]
        ax, ay = (tx-x)*c+(ty-y)*s, -(tx-x)*s+(ty-y)*c
        u, v = dx*c+dy*s, -dx*s+dy*c
        lo, hi = 0.0, 1.0
        for origin, delta, half in ((ax,u,half_length),(ay,v,half_width)):
            if abs(delta) < 1e-10:
                if abs(origin) > half:
                    lo, hi = 1, 0
            else:
                start, end = sorted(((-half-origin)/delta,(half-origin)/delta))
                lo, hi = max(lo, start), min(hi, end)
        if lo <= hi:
            return True
        for px, py in ((ax,ay), (ax+u,ay+v)):
            if hypot(max(0,abs(px)-half_length),max(0,abs(py)-half_width)) <= LINE/2:
                return True
        for px in (-half_length,half_length):
            for py in (-half_width,half_width):
                f = clamp(((px-ax)*u+(py-ay)*v)/length**2, 0, 1)
                if hypot(px-ax-f*u,py-ay-f*v) <= LINE/2:
                    return True
    return False


def segment_index(profile, distance):
    return bisect_right(profile["ends"], distance % profile["length"])


def feedforward_curvatures(profile):
    """Curvaturas assinadas ponderadas pelo ganho de antecipação de cada setor."""
    curvatures = []
    for sector in profile["segments"]:
        gain = sector.get("turn_feedforward", 0.0)
        if not isinstance(gain, (int, float)) or not 0 <= gain <= 1:
            raise ValueError("A antecipação do giro deve ficar entre 0 e 1.")
        curvature = sector.get("curvature")
        if gain and (not isinstance(curvature, (int, float)) or not isfinite(curvature)):
            raise ValueError("O trecho precisa de uma curvatura mapeada para antecipar o giro.")
        curvatures.append(gain * curvature if gain else 0.0)
    return curvatures if any(curvatures) else ()


class SpeedPlan:
    """Calcula o teto de velocidade antes de cada redução do perfil."""
    def __init__(self, profile, braking_limit):
        if braking_limit <= 0:
            raise ValueError("O limite de frenagem planejada deve ser positivo.")
        self.profile = profile
        self.braking = braking_limit
        self.ends = [*profile["ends"], profile["length"]]
        self.speeds = [sector["v"] for sector in profile["segments"]]
        self.entry = self.speeds.copy()
        self.lengths = [end-start for start, end in zip([0, *profile["ends"]], self.ends)]
        for _ in range(2):
            for i in range(len(self.entry)-1, -1, -1):
                next_speed = self.entry[(i+1) % len(self.entry)]
                self.entry[i] = min(self.speeds[i], sqrt(next_speed**2 + 2*self.braking*self.lengths[i]))

    def ceiling(self, distance):
        distance %= self.profile["length"]
        index = bisect_right(self.ends, distance)
        next_speed = self.entry[(index+1) % len(self.entry)]
        return min(self.speeds[index], sqrt(next_speed**2 + 2*self.braking*(self.ends[index]-distance)))

    def expected_time(self, index):
        width, nominal = self.lengths[index], self.speeds[index]
        next_speed = self.entry[(index+1) % len(self.entry)]
        brake_distance = clamp((nominal**2-next_speed**2)/(2*self.braking), 0, width)
        entry_speed = sqrt(next_speed**2 + 2*self.braking*brake_distance)
        return (width-brake_distance)/nominal + (entry_speed-next_speed)/self.braking


def simulate(track, robot, profile=None, seed=7, *, frames=None, trial=None):
    """Simula uma volta e, quando solicitado, coleta poses para a animação."""
    if trial is not None and profile is None:
        raise ValueError("A tomada oficial precisa de um perfil de velocidade.")
    rng = Random(seed)
    initial_s = -trial.launch_offset if trial else 0.0
    x, y, heading = track.pose(initial_s)
    left = right = left_m = right_m = last_error = derivative = lost_time = 0.0
    last_velocity = traction_long = traction_lat = 0.0
    previous_s = initial_s % track.length
    progress = initial_s
    rows, trajectory = [], [(x, y)]
    offsets = robot.sensor_offsets
    sensor_forward = robot.sensor_forward
    wheel_limit = robot.max_wheel_speed
    lateral_limit = robot.effective_lateral_limit
    active_suction = robot.suction_force_n if robot.suction_enabled else 0.0
    limit = ceil((track.length / .14 + 5) / DT)
    stop_at = None
    if trial:
        limit = max(1, int(trial.budget / DT))
        stop_at = trial.stop_distance(profile["length"])
    reason = "volta inválida · limite da tomada de tempo" if trial else "limite de tempo"
    braking = False
    speed_plan = SpeedPlan(profile, robot.braking_limit) if profile else None
    steering_map = feedforward_curvatures(profile) if profile else ()
    yaw_feedforward = 0.0
    # Filtra as transições entre setores mais rapidamente que a resposta do motor.
    feedforward_response = 1 - exp(-DT / (robot.motor_tau / 3))
    mapping_params = {"v": robot.mapping_speed, "kp": robot.mapping_kp, "kd": robot.mapping_kd}
    if speed_plan and robot.acceleration_limit <= 0:
        raise ValueError("O limite de aceleração planejada deve ser positivo.")
    if profile and robot.straight_kp_speed is not None and robot.straight_kp_speed <= 0:
        raise ValueError("A velocidade de referência para o ganho nas retas deve ser positiva.")
    previous_target = 0.0
    previous_odom = encoder_speed = 0.0
    steering_scale = 1.0
    for step in range(limit):
        c, sn = cos(heading), sin(heading)
        readings = []
        for offset in offsets:
            distance, _ = track.nearest(x + sensor_forward*c - offset*sn,
                                        y + sensor_forward*sn + offset*c)
            signal = exp(-.5 * (max(0, distance - LINE/2) / .003)**2)
            readings.append(clamp(signal + rng.gauss(0, .012), 0, 1))
        weights = [max(0, reading - .18) for reading in readings]
        lost = max(readings) < .25
        edge = not lost and max(weights[0], weights[-1]) > .55 * max(weights)
        lost_time = lost_time + DT if lost else 0.0
        error = last_error if lost else sum(a*b for a, b in zip(offsets, weights)) / sum(weights)
        derivative = .84*derivative + .16*(error - last_error)/DT
        last_error = error
        left_ticks = round(left_m / robot.meters_per_tick)
        right_ticks = round(right_m / robot.meters_per_tick)
        odom = (left_ticks + right_ticks) * robot.meters_per_tick / 2
        profile_s = max(0, odom+initial_s)
        index = segment_index(profile, profile_s) if profile else 0
        params = profile["segments"][index] if profile else mapping_params
        planned_speed = params["v"]
        if speed_plan:
            lead = max(0.0, last_velocity)*robot.motor_tau
            planned_speed = min(speed_plan.ceiling(profile_s), speed_plan.ceiling(profile_s+lead))
        speed = min(planned_speed, .20) if lost else planned_speed
        if stop_at is not None:
            remaining = stop_at - profile_s
            braking = braking or remaining <= 0
            speed = 0.0 if braking else min(speed, max(.18, remaining/(3*robot.motor_tau)))
        if speed_plan:
            speed = min(speed, previous_target + robot.acceleration_limit*DT)
            previous_target = speed
        kp = params["kp"]
        if profile:
            encoder_speed += .25 * ((odom-previous_odom)/DT - encoder_speed)
            previous_odom = odom
            if steering_map:
                lookahead_s = profile_s + max(0, encoder_speed) * robot.motor_tau
                target_yaw = encoder_speed * steering_map[segment_index(profile, lookahead_s)]
                yaw_feedforward += feedforward_response * (target_yaw - yaw_feedforward)
                if braking:
                    yaw_feedforward = 0.0
            if robot.straight_kp_speed is not None:
                # Em alta velocidade, uma correção pequena já desloca o robô bastante.
                straightness = clamp(1-abs(params.get("curvature", 1))/.6, 0, 1)
                reference = robot.straight_kp_speed
                speed_scale = max(.35, reference/max(reference, abs(encoder_speed)))
                desired_scale = 1-straightness*(1-speed_scale)
                steering_scale += .22*(desired_scale-steering_scale)
                kp *= steering_scale
        if frames is not None:
            frames.append(Frame(x, y, heading, last_velocity, traction_long, traction_lat,
                                readings, index if profile else None, speed, progress, kp,
                                planned_speed))
        yaw_command = 0.0 if braking else kp * error + params["kd"] * derivative + yaw_feedforward
        target_left = speed - yaw_command * robot.wheelbase / 2
        target_right = speed + yaw_command * robot.wheelbase / 2
        motor_sat = max(abs(target_left), abs(target_right)) > wheel_limit
        response = 1 - exp(-DT / robot.motor_tau)
        left += (clamp(target_left, -wheel_limit, wheel_limit) - left) * response
        right += (clamp(target_right, -wheel_limit, wheel_limit) - right) * response
        if braking and max(abs(left), abs(right)) <= .005:
            left = right = 0.0
        velocity = (left + right) / 2
        wheel_yaw = (right - left) / robot.wheelbase
        demanded_lateral = wheel_yaw * max(abs(velocity), .08)
        longitudinal_accel = (velocity-last_velocity)/DT
        traction_long = longitudinal_accel / lateral_limit
        traction_lat = demanded_lateral / lateral_limit
        yaw_limit = lateral_limit / max(abs(velocity), .08)
        yaw = clamp(wheel_yaw, -yaw_limit, yaw_limit)
        left_m += left * DT
        right_m += right * DT
        heading_mid = heading + yaw * DT / 2
        x += velocity * cos(heading_mid) * DT
        y += velocity * sin(heading_mid) * DT
        heading += yaw * DT
        old_progress = progress
        _, s = track.nearest(x, y, around=previous_s)
        progress += (s - previous_s + track.length/2) % track.length - track.length/2
        previous_s = s
        row = {
            "t_s": (step + 1) * DT,
            "enc_left": round(left_m / robot.meters_per_tick),
            "enc_right": round(right_m / robot.meters_per_tick),
            "gyro_z_rad_s": yaw + robot.gyro_bias + rng.gauss(0, .022),
            "acc_x_m_s2": longitudinal_accel + rng.gauss(0, .08),
            "acc_y_m_s2": velocity*yaw + rng.gauss(0, .08),
            "acc_z_m_s2": GRAVITY + rng.gauss(0, .08),
        }
        if profile:
            on_line = body_over_line(track, x, y, heading, robot.body_length/2, robot.body_width/2,
                                     forward=robot.body_center)
            row.update(line_error_m=error if not lost else "", line_edge=int(edge),
                       line_lost=int(lost), body_on_line=int(on_line), motor_sat=int(motor_sat), target_m_s=speed,
                       profile_target_m_s=params["v"], planned_m_s=planned_speed,
                       encoder_speed_m_s=encoder_speed, steering_kp=kp,
                       track_progress_m=progress, yaw_feedforward_rad_s=yaw_feedforward,
                       lateral_grip_pct=100*abs(traction_lat), lateral_grip_limited=int(abs(traction_lat) > 1),
                       traction_usage_pct=100*hypot(traction_long, traction_lat),
                       traction_long_m_s2=longitudinal_accel, lateral_demand_m_s2=demanded_lateral,
                       lateral_limit_m_s2=lateral_limit, suction_force_n=active_suction,
                       **{f"ir_{i}": value for i, value in enumerate(readings)})
        rows.append(row)
        trajectory.append((x, y))
        last_velocity = velocity
        if trial:
            outcome = trial.observe(track, robot, x, y, heading, old_progress, progress,
                                    row["t_s"], left == right == 0.0 and braking, on_line=on_line)
            if outcome:
                reason = outcome
                break
            continue
        if lost_time >= .40:
            reason = "linha perdida"
            break
        if progress >= track.length - .001:
            reason = "volta completa"
            break
    if frames is not None:
        frames.append(Frame(x, y, heading, last_velocity, traction_long, traction_lat,
                            segment=index if profile else None,
                            target_speed=speed, progress_m=progress, steering_kp=kp,
                            planned_speed=planned_speed))
    return rows, trajectory, reason


def curve_memberships(curvature):
    """Pertinência linear entre os centros das classes, em m⁻¹."""
    magnitude = abs(curvature)
    upper = bisect_right(CURVE_CENTERS, magnitude)
    weights = [0.0] * len(CURVE_CENTERS)
    if upper == len(weights):
        weights[-1] = 1.0
    else:
        low, high = CURVE_CENTERS[upper-1:upper+1]
        weights[upper] = (magnitude-low)/(high-low)
        weights[upper-1] = 1 - weights[upper]
    return weights


def generate(rows, robot, gyro_weight=.75):
    """Integra a odometria e agrupa amostras contíguas pela classe de curva."""
    lateral_limit = robot.effective_lateral_limit
    samples = []
    distance = angle = old_t = old_l = old_r = 0.0
    for row in rows:
        t, left, right = (float(row[key]) for key in ("t_s", "enc_left", "enc_right"))
        dl, dr = (left - old_l) * robot.meters_per_tick, (right - old_r) * robot.meters_per_tick
        dt = t - old_t
        if dt <= 0:
            raise ValueError("Os tempos do CSV devem ser crescentes e começar acima de zero.")
        dtheta = gyro_weight * (float(row["gyro_z_rad_s"]) - robot.gyro_bias) * dt
        dtheta += (1 - gyro_weight) * (dr - dl) / robot.wheelbase
        ds = (dl + dr) / 2
        if ds > 0:
            samples.append((distance, distance + ds, dtheta))
            distance += ds
            angle += dtheta
        old_t, old_l, old_r = t, left, right
    if distance < .4:
        raise ValueError("Telemetria insuficiente para gerar o mapa.")
    count = ceil(distance / .05)
    width = distance / count
    turns = [0.0] * count
    for start, end, turn in samples:
        for i in range(int(start / width), min(count - 1, int(end / width)) + 1):
            turns[i] += turn * max(0, min(end, (i+1)*width) - max(start, i*width)) / (end-start)
    sectors = []
    last_class = (0, "")
    path = [[0.0, 0.0]]
    x = y = heading = 0.0
    for i, turn in enumerate(turns):
        curvature = sum(weight*turns[(i+offset) % count]
                        for offset, weight in zip(range(-2, 3), (1, 2, 3, 2, 1))) / (9*width)
        membership = curve_memberships(curvature)
        level = max(range(len(membership)), key=membership.__getitem__)
        previous_level, previous_side = last_class
        side = "D" if curvature > 0 else "E"
        if (previous_level == 0 or previous_side == side) and membership[previous_level] + .14 >= membership[level]:
            level = previous_level
        classification = (level, side if level else "")
        if not sectors or classification != last_class:
            sectors.append((i, classification, [], []))
        sectors[-1][2].append(curvature)
        sectors[-1][3].append(membership)
        last_class = classification
        x += width * cos(heading + turn/2)
        y += width * sin(heading + turn/2)
        heading += turn
        path.append([x, y])
    segments, ends, path_sections = [], [], []
    for start, (level, side), curvatures, memberships in sectors:
        end = start + len(curvatures)
        curvature = sum(curvatures) / len(curvatures)
        peak = max(abs(value) for value in curvatures)
        speed = min(.88 * robot.max_wheel_speed,
                    sqrt(.55 * lateral_limit / max(peak, .1)),
                    .85 * robot.max_wheel_speed / (1 + peak*robot.wheelbase/2))
        label = f"{side}{CURVE_LEVELS[level]}" if side else "Reta"
        segments.append({"label": label, "curvature": curvature, "curvature_peak": peak,
                         "membership": dict(zip(CURVE_LEVELS,
                                                (sum(values)/len(values) for values in zip(*memberships)))),
                         "v": speed, "kp": 92 + min(22, 4*peak), "kd": 6.5, "turn_feedforward": 0.0})
        ends.append(end * width)
        path_sections.append(path[start:end+1])
    return {"length": distance, "ends": ends[:-1], "segments": segments, "path": path,
            "path_sections": path_sections, "heading_rad": angle, "version": 2}


def diagnose(rows, profile, robot, complete, *, excluded_sectors=(), search_speed=True):
    """Agrupa pela mesma odometria usada pelo controlador e propõe uma busca local."""
    from copy import deepcopy

    proposed = deepcopy(profile)
    lateral_limit = robot.effective_lateral_limit
    groups = [[] for _ in profile["segments"]]
    last_t = last_s = 0.0
    for row in rows:
        t = float(row["t_s"])
        s = (float(row["enc_left"]) + float(row["enc_right"])) * robot.meters_per_tick / 2
        index = segment_index(profile, last_s)
        groups[index].append((row, t - last_t))
        last_t, last_s = t, s
    results = []
    bounds = [0, *profile["ends"], profile["length"]]
    plan = SpeedPlan(profile, robot.braking_limit) if rows and "planned_m_s" in rows[0] else None
    racing = complete and bool(rows) and all(int(row.get("body_on_line", 1)) for row in rows)
    candidates = []
    for index, group in enumerate(groups):
        old, new = profile["segments"][index], proposed["segments"][index]
        elapsed = sum(dt for _, dt in group)
        width = bounds[index+1] - bounds[index]
        expected = plan.expected_time(index) if plan else width / old["v"]
        rms = sqrt(sum(float(r["line_error_m"])**2 * dt for r, dt in group
                       if r["line_error_m"] != "") /
                   max(DT, sum(dt for r, dt in group if r["line_error_m"] != "")))
        flags = {key: sum(int(r[key])*dt for r, dt in group)/max(DT, elapsed)
                 for key in ("line_edge", "line_lost", "motor_sat")}
        flags["body_off_line"] = sum((1-int(r.get("body_on_line", 1)))*dt
                                     for r, dt in group)/max(DT, elapsed)
        lateral = max((abs(float(r["acc_y_m_s2"])) for r, _ in group), default=0)
        partial = not complete and index == segment_index(profile, last_s)
        covered = elapsed > 0 and not partial
        delay = elapsed - expected if covered else None
        if racing:
            action = "manter" if group else "sem amostras"
            faster = min(.96*robot.max_wheel_speed, old["v"]*1.07)
            if (search_speed and index not in excluded_sectors and covered and width >= .10
                    and faster > old["v"] and not flags["line_lost"]
                    and flags["line_edge"] <= .2 and flags["motor_sat"] < .05
                    and lateral < .80*lateral_limit and rms <= .012):
                candidates.append((index, faster))
        elif flags["body_off_line"]:
            action = "corpo saiu da faixa"
            new["v"] *= .82
        elif flags["line_lost"] > .03 or flags["line_edge"] > .05:
            action = "perda de linha" if flags["line_lost"] > .03 else "sensor na borda"
            new["v"] *= .82
            new["kd"] = min(11, old["kd"] * 1.07)
        elif lateral > .88*lateral_limit or flags["motor_sat"] > .10:
            action = "limite lateral ou motor"
            new["v"] *= .91
        elif covered and rms > .012:
            action = "erro lateral"
            new["kp"] = min(135, old["kp"] * 1.06)
        elif covered and rms < .008 and lateral < .72*lateral_limit:
            action = "testar mais velocidade"
            new["v"] = min(.96*robot.max_wheel_speed, old["v"] * 1.05)
        else:
            action = "manter" if group else "sem amostras"
        if not racing:
            new["v"] = max(.22, new["v"])
        results.append({"trecho": index+1, "nota": old.get("label", ""), "comprimento_m": width,
                        "tempo_s": elapsed, "esperado_s": expected,
                        "saldo_s": delay, "erro_rms_m": rms, "pico_lateral_m_s2": lateral,
                        **flags, "acao": action, "v_anterior_m_s": old["v"], "v_proposta_m_s": new["v"],
                        "kp_proposto": new["kp"], "kd_proposto": new["kd"]})
    if candidates:
        reference_plan = plan or SpeedPlan(profile, robot.braking_limit)
        baseline = sum(reference_plan.expected_time(i) for i in range(len(groups)))
        best_gain = .002
        best = None
        for index, speed in candidates:
            sector = proposed["segments"][index]
            nominal = sector["v"]
            sector["v"] = speed
            candidate_plan = SpeedPlan(proposed, robot.braking_limit)
            gain = baseline - sum(candidate_plan.expected_time(i) for i in range(len(groups)))
            sector["v"] = nominal
            if gain > best_gain:
                best_gain, best = gain, (index, speed)
        if best is not None:
            index, speed = best
            proposed["segments"][index]["v"] = speed
            results[index].update(acao="testar mais velocidade", v_proposta_m_s=speed)
    return proposed, results
