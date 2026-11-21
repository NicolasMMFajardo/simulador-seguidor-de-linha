"""Editor e bancada local para o seguidor de linha."""
import tkinter as tk
from bisect import bisect_left
from colorsys import hsv_to_rgb
from concurrent.futures import Future
from dataclasses import asdict, dataclass, field
from datetime import datetime
from itertools import groupby
import json
from math import atan2, cos, floor, hypot, isfinite, log10, sin
from pathlib import Path
from queue import Empty, Queue
from tempfile import NamedTemporaryFile
from threading import Event, Thread
from time import monotonic
from tkinter import filedialog, messagebox, ttk
from zipfile import ZIP_DEFLATED, ZipFile

from offboard import describe_proposal, diagnose_pass, read_csv, read_json, write_csv, write_json
from optimization import AutoOptimizer
from paired_validation import PAIR_SEEDS, SCREENING_SEED, ValidationMetrics, run_pairs
from parameter_editor import ParameterEditor
from regulation import (FINISH_GAP, MAX_ATTEMPTS, SESSION_SECONDS, VALID_LAP, ZONE_HALF_WIDTH,
                        OfficialTrial, calibrate_stop_odom, curvature_markers,
                        pro_example, track_bends, track_checks)
from simulator import DT, LINE, Frame, Robot, Track, generate, simulate

BG = "#1a1717"
PANEL = "#262626"
CANVAS_BG = "#211e1e"
BORDER = "#474747"
BRAND_DEEP = "#5b0c0c"
BRAND = "#a81b1b"
BRAND_BRIGHT = "#fd0606"
INK = "#fff8f6"
MUTED = "#c5b8b6"
TRACE = "#f3bdba"
ACCENT = "#60e3ba"
GRIP_WARNING = "#ffc768"
GRIP_LIMITED = "#ff6c74"
CURVE_COLORS = {"Reta": "#a5b5c7", "5": "#60e3ba", "4": "#6fcfff",
                "3": "#d1a0ff", "2": "#ffbe6b", "1": "#ff777d"}
WIDTH, HEIGHT = 840, 490
SCALE = 290
MARGIN = 28
DETAIL_WIDTH = 210
SHORT_NOTE_M = .15
DEFAULT_TRACK_PATH = Path(__file__).resolve().with_name("pista-do-print-pro.json")


def display_weights(segment):
    return tuple(f"{segment.get(key, 0):.{digits}f}".replace(".", ",")
                 for key, digits in (("v", 2), ("kp", 1), ("kd", 1), ("turn_feedforward", 2)))


def map_note_groups(profile, *, technical=False):
    """Retorna intervalos de setores para o desenho, sem alterar o perfil."""
    notes = [segment.get("label", "") for segment in profile["segments"]]
    if technical:
        return [(index, index+1, note) for index, note in enumerate(notes)]
    bounds = [0, *profile["ends"], profile["length"]]
    lengths = [end-start for start, end in zip(bounds, bounds[1:])]
    groups = []
    for side, region in groupby(range(len(notes)), key=lambda i: notes[i][:1]):
        indices = list(region)
        if side not in ("D", "E"):
            groups.extend((index, index+1, notes[index]) for index in indices)
            continue
        anchors = [index for index in indices if lengths[index] > SHORT_NOTE_M + 1e-8]
        if not anchors:
            anchors = [max(indices, key=lambda i: (lengths[i], -int(notes[i][1:])))]

        def visual_note(index):
            center = (bounds[index]+bounds[index+1])/2
            anchor = min(anchors, key=lambda i: (max(bounds[i]-center, center-bounds[i+1], 0),
                                                 int(notes[i][1:])))
            return notes[anchor]

        for note, section in groupby(indices, key=visual_note):
            members = list(section)
            groups.append((members[0], members[-1]+1, note))
    return groups


def track_section(track, start, end):
    points = []
    for offset, (x, y, dx, dy, length) in zip(track.starts, track.segments):
        if offset >= end:
            break
        if offset + length <= start:
            continue
        if not points:
            fraction = max(0, (start-offset)/length)
            points.append((x+dx*fraction, y+dy*fraction))
        fraction = min(1, (end-offset)/length)
        points.append((x+dx*fraction, y+dy*fraction))
    return points


@dataclass
class Iteration:
    session: int
    profile: dict
    reference_id: int | None
    metrics: ValidationMetrics | None = None
    decision: str = "Aguardando validação"
    diagnosed: bool = False
    validation_csv: str | None = None
    diagnostic_csv: str | None = None
    curve_csv: str | None = None
    applied_proposal_json: str | None = None
    passage: "Passage | None" = None
    confirmation: dict | None = None
    seed_metrics: dict[int, ValidationMetrics] = field(default_factory=dict)
    automatic: bool = False


@dataclass(frozen=True)
class Passage:
    distances: tuple[float, ...]
    times: tuple[float, ...]

    @classmethod
    def from_frames(cls, frames, start_time, finish_time, finish_distance):
        distances, times = [0.0], [0.0]
        for index, frame in enumerate(frames):
            time = index*DT
            if time <= start_time:
                continue
            end = finish_time is not None and time >= finish_time
            distance = finish_distance if end else min(frame.progress_m, finish_distance)
            if distance > distances[-1]:
                distances.append(distance)
                times.append((finish_time if end else time)-start_time)
            if end:
                break
        return cls(tuple(distances), tuple(times))

    def time_at(self, distance):
        index = bisect_left(self.distances, distance)
        if index == len(self.distances):
            return None
        if index == 0:
            return 0.0
        left, right = self.distances[index-1:index+1]
        fraction = (distance-left)/(right-left)
        before, after = self.times[index-1:index+1]
        return before + fraction*(after-before)


def simulate_pass(points, robot, profile, seed, trial=None):
    track = Track(points)
    frames = []
    rows, trajectory, reason = simulate(track, robot, profile, seed, frames=frames, trial=trial)
    return track, frames, rows, trajectory, reason


class App:
    def __init__(self, root):
        self.root = root
        root.title("Team Daedalus · Bancada de simulação")
        root.configure(bg=BG)
        root.minsize(915, 735)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", padding=(9, 6), background=PANEL, foreground=INK,
                        bordercolor=BORDER, lightcolor=PANEL, darkcolor=PANEL, focuscolor=BRAND_BRIGHT)
        style.map("TButton", background=[("disabled", BG), ("pressed", BRAND), ("active", BRAND_DEEP)],
                  foreground=[("disabled", "#958787")])
        style.configure("Brand.TButton", background=BRAND_DEEP, bordercolor=BRAND,
                        lightcolor=BRAND_DEEP, darkcolor=BRAND_DEEP)
        style.map("Brand.TButton", background=[("disabled", PANEL), ("pressed", BRAND_DEEP), ("active", BRAND)],
                  bordercolor=[("disabled", BORDER)])
        style.configure("Speed.Toolbutton", padding=(7, 5), background=PANEL, foreground=INK,
                        bordercolor=BORDER, lightcolor=PANEL, darkcolor=PANEL, focuscolor=BRAND_BRIGHT)
        style.map("Speed.Toolbutton", background=[("disabled", BG), ("selected", BRAND_DEEP), ("active", BORDER)],
                  bordercolor=[("disabled", BORDER), ("selected", BRAND)],
                  foreground=[("disabled", "#958787")])
        style.configure("TNotebook", background=BG, borderwidth=0,
                        bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
        style.configure("TNotebook.Tab", background=PANEL, foreground=MUTED, padding=(12, 7),
                        bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
        style.map("TNotebook.Tab", background=[("selected", BRAND_DEEP), ("active", BORDER)],
                  foreground=[("selected", INK)], bordercolor=[("selected", BRAND)])
        style.configure("Treeview", background=PANEL, fieldbackground=PANEL, foreground=INK,
                        bordercolor=BORDER, rowheight=25)
        style.map("Treeview", background=[("selected", BRAND_DEEP)], foreground=[("selected", INK)])
        style.configure("Treeview.Heading", background=BRAND_DEEP, foreground=INK,
                        bordercolor=BORDER, lightcolor=BRAND_DEEP, darkcolor=BRAND_DEEP)
        style.map("Treeview.Heading", background=[("active", BRAND)])
        style.configure("Vertical.TScrollbar", background=BORDER, troughcolor=BG, arrowcolor=INK,
                        bordercolor=BG, lightcolor=BORDER, darkcolor=BORDER)
        style.configure("Map.TCheckbutton", background=PANEL, foreground=INK)
        style.map("Map.TCheckbutton", background=[("active", BRAND_DEEP)])
        style.configure("Parameters.TFrame", background=PANEL)
        style.configure("Parameters.TLabel", background=PANEL, foreground=INK)
        style.configure("Parameters.TLabelframe", background=PANEL, bordercolor=BORDER,
                        lightcolor=BORDER, darkcolor=BORDER)
        style.configure("Parameters.TLabelframe.Label", background=PANEL, foreground=TRACE)
        style.configure("Parameters.TCheckbutton", background=PANEL, foreground=INK)
        style.map("Parameters.TCheckbutton", background=[("active", BRAND_DEEP)],
                  foreground=[("disabled", MUTED)])
        style.configure("Parameters.TEntry", background=PANEL, fieldbackground=CANVAS_BG,
                        bordercolor=BORDER, foreground=INK, insertcolor=INK, padding=(4, 1))
        style.map("Parameters.TEntry", foreground=[("disabled", MUTED)], fieldbackground=[("disabled", PANEL)],
                  bordercolor=[("focus", BRAND)])

        self.robot = Robot()
        self.points = read_json(DEFAULT_TRACK_PATH)["points_m"]
        track = Track(self.points)
        self.bends = track_bends(track)
        self.track_length = track.length
        self.trajectory = []
        self.frames = []
        self.frame_index = 0
        self.playback_started = None
        self.playback_kind = None
        self.playback_job = None
        self.playback_offset = 0.0
        self.playback_rate = 1
        self.run_summary = ""
        self.run_sections = []
        self.profile = self.proposal = self.best = self.mapping = self.validation = None
        self.proposal_file = None
        self.best_score = None
        self.best_stop_odom_m = None
        self.iterations = []
        self.map_history_start = 0
        self.confirmation_stop = None
        self.confirmation_job = None
        self.confirmation_progress = ""
        self.optimizer = AutoOptimizer(self)
        self.reference_id = self.pending_id = self.last_iteration_id = None
        self.reference_rows = None
        self.latest_validation_id = None
        self.live_reference = None
        self.live_passage = None
        self.live_start = self.live_finish = None
        self.official_reference = None
        self.report, self.curve_report = [], []
        self.drawing = False
        self.drawing_stroke = False
        self.zoom, self.pan_x, self.pan_y = 1.0, 0.0, 0.0
        self.pan_anchor = None
        self.session, self.number = 1, 0
        self.output = Path(__file__).resolve().parent / "resultados"
        self.opened_at = datetime.now().astimezone()
        self.track_loaded_at = self.opened_at
        self.session_folder = None
        self.official_started = None
        self.official_round = 1
        self.official_attempts = []
        self.official_geometry = None

        header = tk.Frame(root, bg=BRAND_DEEP)
        header.pack(fill="x")
        tk.Frame(header, bg=BRAND_BRIGHT, height=3).pack(side="top", fill="x")
        logo_source = tk.PhotoImage(file=Path(__file__).resolve().parent / "assets" / "team-daedalus.png")
        self.brand_logo = tk.PhotoImage()
        header.tk.call(str(self.brand_logo), "copy", str(logo_source),
                       "-from", 346, 350, 1937, 1368, "-subsample", 13, 13)
        root.iconphoto(True, self.brand_logo)
        tk.Label(header, image=self.brand_logo, bg=BRAND_DEEP, borderwidth=0).pack(
            side="left", padx=(21, 18), pady=4)
        title = tk.Frame(header, bg=BRAND_DEEP)
        title.pack(side="left")
        tk.Label(title, text="Bancada de simulação", bg=BRAND_DEEP, fg=INK,
                 font=("Segoe UI", 23, "bold")).pack(anchor="w")
        tk.Label(title, text="Seguidor de linha · encoder duplo, refletância e IMU",
                 bg=BRAND_DEEP, fg=TRACE, font=("Segoe UI", 10)).pack(anchor="w", pady=(0, 4))
        toolbar = tk.Frame(root, bg=BG)
        toolbar.pack(fill="x", padx=18, pady=(8, 0))
        self.track_buttons = []
        for label, action in (("Desenhar", self.start_drawing), ("Exemplo", self.load_example),
                              ("Abrir pista", self.open_track), ("Salvar pista", self.save_track)):
            button = ttk.Button(toolbar, text=label, command=action)
            button.pack(side="left", padx=3)
            self.track_buttons.append(button)
        self.export_button = ttk.Button(toolbar, text="Exportar sessão", command=self.export_session,
                                        style="Brand.TButton")
        self.export_button.pack(side="right", padx=3)
        self.optimize_button = ttk.Button(toolbar, text="Otimizar automaticamente",
                                          command=self.toggle_optimization, style="Brand.TButton")
        self.optimize_button.pack(side="right", padx=3)
        for label, action in (("−", lambda: self.zoom_track(1/1.25)),
                              ("+", lambda: self.zoom_track(1.25)),
                              ("Ajustar vista", self.reset_view)):
            ttk.Button(toolbar, text=label, width=3 if len(label) == 1 else 13,
                       command=action).pack(side="left", padx=3)
        self.notebook = ttk.Notebook(root)
        self.track_panel = tk.Frame(self.notebook, bg=CANVAS_BG)
        self.sidebar_panel = tk.Frame(self.track_panel, bg=BORDER)
        self.sidebar_view = tk.Canvas(self.sidebar_panel, width=DETAIL_WIDTH, bg=CANVAS_BG,
                                      highlightthickness=0, yscrollincrement=16)
        self.sidebar_view.pack(side="left", fill="both", expand=True, padx=(1, 0))
        sidebar_scroll = ttk.Scrollbar(self.sidebar_panel, command=self.sidebar_view.yview)
        sidebar_scroll.pack(side="right", fill="y")
        self.sidebar_view.configure(yscrollcommand=sidebar_scroll.set)
        self.sidebar_content = tk.Frame(self.sidebar_view, bg=CANVAS_BG)
        sidebar_window = self.sidebar_view.create_window(0, 0, anchor="nw", window=self.sidebar_content)
        self.sidebar_view.bind("<Configure>", lambda event: self.sidebar_view.itemconfigure(
            sidebar_window, width=event.width))
        self.sidebar_content.bind("<Configure>", lambda _: self.sidebar_view.configure(
            scrollregion=self.sidebar_view.bbox(sidebar_window)))
        self.telemetry_canvas = tk.Canvas(self.sidebar_content, width=DETAIL_WIDTH, height=372,
                                          bg=CANVAS_BG, highlightthickness=0)
        self.telemetry_canvas.pack(fill="x")
        self.sidebar_panel.pack(side="right", fill="y")
        self.track_view = tk.Frame(self.track_panel, bg=CANVAS_BG)
        self.track_view.pack(side="left", fill="both", expand=True)
        curve_bar = tk.Frame(self.track_view, bg=PANEL)
        curve_bar.pack(fill="x")
        self.curve_counter = tk.StringVar()
        self.curve_state = tk.StringVar()
        tk.Label(curve_bar, textvariable=self.curve_counter, bg=BRAND_DEEP, fg=INK,
                 font=("Segoe UI", 10, "bold"), padx=10, pady=3).pack(side="left")
        tk.Label(curve_bar, textvariable=self.curve_state, bg=PANEL, fg=MUTED,
                 padx=8).pack(side="left")
        self.canvas = tk.Canvas(self.track_view, width=WIDTH-DETAIL_WIDTH, height=HEIGHT,
                                bg=CANVAS_BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.map_panel = tk.Frame(self.notebook, bg=CANVAS_BG)
        map_toolbar = tk.Frame(self.map_panel, bg=PANEL)
        map_toolbar.pack(fill="x")
        self.map_technical = tk.BooleanVar(value=False)
        self.map_notes_count = tk.StringVar()
        ttk.Checkbutton(map_toolbar, text="Mostrar setores técnicos", variable=self.map_technical,
                        command=self.draw_map, style="Map.TCheckbutton").pack(side="left", padx=12, pady=6)
        tk.Label(map_toolbar, textvariable=self.map_notes_count, bg=PANEL, fg=MUTED).pack(
            side="right", padx=12, pady=6)
        self.map_canvas = tk.Canvas(self.map_panel, bg=CANVAS_BG, highlightthickness=0)
        self.map_canvas.pack(fill="both", expand=True)
        self.notebook.add(self.track_panel, text="Pista e trajetória")
        self.notebook.add(self.map_panel, text="Mapa estimado")
        table_panel = tk.Frame(self.notebook, bg=BG)
        self.notebook.add(table_panel, text="Diagnóstico")
        self.diagnostic_context = tk.StringVar(value="Valide uma passada e clique em Diagnosticar.")
        tk.Label(table_panel, textvariable=self.diagnostic_context, bg=BG, fg=INK, anchor="w",
                 justify="left", wraplength=850, padx=10, pady=6).pack(fill="x")
        self.diagnostic_views = ttk.Notebook(table_panel)
        self.diagnostic_views.pack(fill="both", expand=True)
        curves_panel = tk.Frame(self.diagnostic_views, bg=BG)
        technical_panel = tk.Frame(self.diagnostic_views, bg=BG)
        self.diagnostic_views.add(curves_panel, text="Curvas e saídas")
        self.diagnostic_views.add(technical_panel, text="Setores técnicos")
        fields = ("Região", "Parcial (s)", "Δ local (s)", "V. saída (m/s)", "Limitante provável", "Ajuste a investigar")
        self.curve_table = ttk.Treeview(curves_panel, columns=fields, show="headings")
        for label, width in zip(fields, (123, 82, 82, 94, 183, 247)):
            self.curve_table.heading(label, text=label)
            self.curve_table.column(label, width=width, minwidth=width, anchor="center")
        self.curve_table.tag_configure("proposta", foreground=ACCENT)
        self.curve_evidence = tk.StringVar()
        tk.Label(curves_panel, textvariable=self.curve_evidence, bg=PANEL, fg=MUTED, anchor="w",
                 justify="left", wraplength=840, padx=10, pady=4).pack(side="bottom", fill="x")
        self.curve_table.bind("<<TreeviewSelect>>", self.show_curve_evidence)
        yscroll = ttk.Scrollbar(curves_panel, command=self.curve_table.yview)
        xscroll = ttk.Scrollbar(curves_panel, orient="horizontal", command=self.curve_table.xview)
        self.curve_table.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        yscroll.pack(side="right", fill="y")
        xscroll.pack(side="bottom", fill="x")
        self.curve_table.pack(fill="both", expand=True)
        columns = ("Trecho", "Erro RMS mm", "Borda %", "Perda QTR %", "Corpo fora %", "Motor %",
                   "Pico lateral", "Saldo s", "Vel. proposta", "Ação")
        self.table = ttk.Treeview(technical_panel, columns=columns, show="headings")
        scrollbar = ttk.Scrollbar(technical_panel, command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.table.pack(fill="both", expand=True)
        for label, width in zip(columns, (60, 82, 64, 78, 79, 63, 83, 67, 86, 140)):
            self.table.heading(label, text=label)
            self.table.column(label, width=width, anchor="center")

        weights_panel = tk.Frame(self.notebook, bg=BG)
        self.notebook.add(weights_panel, text="Trechos e pesos")
        tk.Label(weights_panel, text="Trechos do último teste com os mesmos parâmetros de velocidade e direção",
                 bg=BG, fg=MUTED, anchor="w", padx=12, pady=10).pack(fill="x")
        columns = ("Grupo", "Trechos", "Nominal (m/s)", "Kp", "Kd", "Antecipação")
        self.weights_table = ttk.Treeview(weights_panel, columns=columns, show="headings")
        scrollbar = ttk.Scrollbar(weights_panel, command=self.weights_table.yview)
        self.weights_table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.weights_table.pack(fill="both", expand=True)
        for label, width in zip(columns, (75, 310, 110, 75, 75, 95)):
            self.weights_table.heading(label, text=label)
            self.weights_table.column(label, width=width, anchor="center")

        history_panel = tk.Frame(self.notebook, bg=BG)
        self.notebook.add(history_panel, text="Histórico")
        tk.Label(history_panel, text="Triagem na semente 7 · confirmação em 3 pares · voltas válidas da candidata e Δ mediano",
                 bg=BG, fg=MUTED, anchor="w", padx=12, pady=10).pack(fill="x")
        columns = ("Iteração", "Sessão", "Ref.", "Volta", "Tempo (s)", "Δ tempo (s)",
                   "Dist. (m)", "Perda QTR %", "Corpo fora %", "Diagnóstico", "Confirmação", "Decisão")
        self.history_table = ttk.Treeview(history_panel, columns=columns, show="headings")
        scrollbar = ttk.Scrollbar(history_panel, command=self.history_table.yview)
        horizontal = ttk.Scrollbar(history_panel, orient="horizontal", command=self.history_table.xview)
        self.history_table.configure(yscrollcommand=scrollbar.set, xscrollcommand=horizontal.set)
        self.pair_details = tk.StringVar()
        tk.Label(history_panel, textvariable=self.pair_details, anchor="w", justify="left",
                 wraplength=830, bg=PANEL, fg=TRACE, padx=10, pady=5).pack(side="bottom", fill="x")
        horizontal.pack(side="bottom", fill="x")
        self.history_table.bind("<<TreeviewSelect>>", lambda _: self.show_pair_details())
        scrollbar.pack(side="right", fill="y")
        self.history_table.pack(fill="both", expand=True)
        for label, width in zip(columns, (60, 53, 42, 71, 73, 78, 67, 77, 78, 80, 145, 130)):
            self.history_table.heading(label, text=label)
            self.history_table.column(label, width=width, minwidth=40, anchor="center")
        self.history_table.tag_configure("accepted", foreground=ACCENT)
        self.history_table.tag_configure("rejected", foreground="#ffab8c")

        rules_panel = tk.Frame(self.notebook, bg=BG)
        self.notebook.add(rules_panel, text="RoboCore Pro")
        rules_toolbar = tk.Frame(rules_panel, bg=BG)
        rules_toolbar.pack(fill="x", padx=10, pady=8)
        self.rules_buttons = []
        for label, action in (("Exemplo Pro", self.load_pro_example),
                              ("Conferir geometria", self.check_rules),
                              ("Nova tomada de tempo", self.start_official_round)):
            button = ttk.Button(rules_toolbar, text=label, command=lambda fn=action: self.safe(fn))
            button.pack(side="left", padx=3)
            self.rules_buttons.append(button)
        self.rules_text = tk.Text(rules_panel, height=12, wrap="word", background=PANEL, foreground=INK,
                                  padx=15, pady=12, relief="flat", font=("Segoe UI", 11))
        scrollbar = ttk.Scrollbar(rules_panel, command=self.rules_text.yview)
        scrollbar.pack(side="right", fill="y", padx=(0, 10), pady=(0, 12))
        self.rules_text.configure(yscrollcommand=scrollbar.set)
        self.rules_text.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self.render_rules()
        self.parameter_editor = ParameterEditor(self.notebook, self.safe, self.apply_robot_parameters,
                                                self.prepare_manual_profile)
        self.notebook.add(self.parameter_editor.robot_page, text="Robô")
        self.notebook.add(self.parameter_editor, text="Parâmetros")

        workflow = tk.Frame(root, bg=BG)
        self.buttons = {}
        for key, label, action in (
                ("map", "1  Mapear", self.run_mapping),
                ("generate", "Gerar mapa", self.generate_map),
                ("validate", "2  Validar", self.run_validation),
                ("diagnose", "Diagnosticar", self.run_diagnosis),
                ("apply", "Aplicar proposta", self.apply_proposal),
                ("timed", "3  Tomar tempo", self.run_timed)):
            button = ttk.Button(workflow, text=label, command=lambda fn=action: self.safe(fn),
                                style="Brand.TButton" if key in ("map", "validate", "timed") else "TButton")
            button.pack(side="left", padx=3)
            self.buttons[key] = button
        speed_controls = tk.Frame(workflow, bg=BG)
        speed_controls.pack(side="right", padx=(8, 3))
        self.speed_label = tk.Label(speed_controls, text="Animação", fg=MUTED, bg=BG)
        self.speed_label.pack(side="left", padx=(0, 6))
        self.playback_preference = tk.IntVar(value=1)
        self.speed_buttons = []
        for rate in (1, 2, 4, 8):
            button = ttk.Radiobutton(speed_controls, text=f"{rate}×", value=rate,
                                     variable=self.playback_preference, style="Speed.Toolbutton",
                                     command=self.change_playback_rate)
            button.pack(side="left", padx=1)
            self.speed_buttons.append(button)
        indicators = tk.Frame(root, bg=PANEL)
        self.iteration_indicator = tk.StringVar()
        self.reference_indicator = tk.StringVar()
        self.comparison_indicator = tk.StringVar()
        sidebar_summary = tk.Frame(self.sidebar_content, bg=PANEL, highlightthickness=1,
                                   highlightbackground=BORDER)
        sidebar_summary.pack(fill="x", padx=(11, 12), pady=(5, 12))
        for column, value in enumerate((self.iteration_indicator, self.reference_indicator,
                                        self.comparison_indicator)):
            indicators.columnconfigure(column, weight=1)
            tk.Label(indicators, textvariable=value, bg=PANEL, fg=INK,
                     font=("Segoe UI", 10), anchor="w", padx=10, pady=7).grid(
                         row=0, column=column, sticky="ew")
            tk.Label(sidebar_summary, textvariable=value, bg=PANEL, fg=INK if column < 2 else MUTED,
                     font=("Segoe UI", 9, "bold" if column == 0 else "normal"),
                     anchor="w", justify="left", wraplength=DETAIL_WIDTH-48).pack(
                         fill="x", padx=10, pady=(7 if column == 0 else 3, 0))
        self.sidebar_history_button = ttk.Button(sidebar_summary, text="Ver histórico",
                                                 command=lambda: self.notebook.select(history_panel))
        self.sidebar_history_button.pack(fill="x", padx=9, pady=(8, 8))
        ttk.Button(indicators, text="Ver histórico",
                   command=lambda: self.notebook.select(history_panel)).grid(row=0, column=3, padx=5)
        self.status = tk.StringVar(value="Pista Pro regularizada carregada. Comece pelo mapeamento.")
        ttk.Label(root, textvariable=self.status, background=BG, foreground=TRACE,
                  wraplength=850, padding=(0, 9)).pack(side="bottom", fill="x", padx=21)
        indicators.pack(side="bottom", fill="x", padx=21, pady=(7, 0))
        workflow.pack(side="bottom", fill="x", padx=18)
        self.notebook.pack(fill="both", expand=True, padx=19, pady=9)
        self.canvas.bind("<Configure>", lambda _: self.draw_track())
        self.map_canvas.bind("<Configure>", lambda _: self.draw_map())
        self.canvas.bind("<Button-1>", self.start_stroke)
        self.canvas.bind("<B1-Motion>", self.add_point)
        self.canvas.bind("<ButtonRelease-1>", self.finish_drawing)
        self.canvas.bind("<MouseWheel>", self.zoom_wheel)
        self.canvas.bind("<Button-4>", lambda e: self.zoom_track(1.25, e.x, e.y))
        self.canvas.bind("<Button-5>", lambda e: self.zoom_track(1/1.25, e.x, e.y))
        sidebar_widgets = (self.sidebar_view, self.sidebar_content, self.telemetry_canvas,
                           sidebar_summary, *sidebar_summary.winfo_children())
        for widget in sidebar_widgets:
            widget.bind("<MouseWheel>", self.scroll_sidebar)
            widget.bind("<Button-4>", lambda _: self.scroll_sidebar_by(-3))
            widget.bind("<Button-5>", lambda _: self.scroll_sidebar_by(3))
        for button in (2, 3):
            self.canvas.bind(f"<ButtonPress-{button}>", self.begin_pan)
            self.canvas.bind(f"<B{button}-Motion>", self.move_pan)
            self.canvas.bind(f"<ButtonRelease-{button}>", self.end_pan)
        self.update_curve_counter()
        self.update_buttons()
        root.bind("<Destroy>", self.stop_confirmation_on_close, add="+")

    def safe(self, action):
        try:
            action()
        except (ValueError, OSError) as error:
            messagebox.showerror("Simulador", str(error))
        finally:
            self.update_buttons()

    def update_buttons(self):
        playing = self.playback_started is not None
        optimizing = self.optimizer.active
        busy = optimizing or playing or self.confirmation_stop is not None
        if optimizing:
            self.optimize_button.configure(text="Pausa solicitada" if self.optimizer.stopping else "Pausar otimização",
                                           state="disabled" if self.optimizer.stopping else "normal")
        else:
            self.optimize_button.configure(text="Otimizar automaticamente",
                                           state="normal" if self.profile is not None and not busy else "disabled")
        official_playback = playing and self.playback_kind == "tomada-de-tempo"
        enabled = dict(map=bool(self.points) and not self.drawing, generate=self.mapping is not None,
                       validate=self.profile is not None, diagnose=self.validation is not None,
                       apply=self.proposal is not None and self.reference_id is not None,
                       timed=self.best is not None and self.pending_id is None
                       and len(self.official_attempts) < MAX_ATTEMPTS)
        for key, button in self.buttons.items():
            button.configure(state="normal" if enabled[key] and not busy else "disabled")
        for button in self.track_buttons + self.rules_buttons:
            button.configure(state="disabled" if busy else "normal")
        for button in self.speed_buttons:
            button.state(("disabled",) if official_playback or optimizing else ("!disabled",))
        self.speed_label.configure(text="Pro · 1×" if official_playback else "Animação")
        self.export_button.configure(state="normal" if self.session_folder and not busy else "disabled")
        self.parameter_editor.sync(self.robot, self.profile,
                                   robot_editable=not busy and not self.drawing,
                                   profile_editable=not busy and self.pending_id is None and self.profile is not None)
        self.update_iteration_indicator()

    def toggle_optimization(self):
        if self.optimizer.active:
            self.optimizer.pause()
        elif self.parameter_editor.has_unsaved_profile():
            self.status.set("Há alterações em Parâmetros. Prepare a validação ou descarte o rascunho para otimizar.")
        else:
            self.optimizer.start()

    def update_iteration_indicator(self):
        validating = self.playback_started is not None and self.playback_kind == "validacao"
        latest_id = self.latest_validation_id
        active_id = self.pending_id if self.pending_id is not None else latest_id
        latest = self.iterations[latest_id] if latest_id is not None else None
        best_id = latest.reference_id if validating and latest else self.reference_id
        current = "—" if active_id is None else f"#{active_id+1:02d}"
        if self.confirmation_stop is not None:
            current += " · confirmando"
        elif validating:
            current += " · validando"
        elif self.pending_id is not None:
            current += " · aguardando teste"
        self.iteration_indicator.set(f"Iteração {current}")

        reference_text = "Melhor · —"
        if best_id is not None:
            metrics = self.iterations[best_id].metrics
            measure = f"{metrics.duration:.2f} s" if metrics.complete else f"{metrics.distance:.2f} m alcançados"
            reference_text = f"Melhor · #{best_id+1:02d} · {measure.replace('.', ',')}"
        self.reference_indicator.set(reference_text)

        comparison = "Resultado pendente" if validating else "Sem comparação"
        if latest and not validating:
            comparison = f"Última · {latest.decision}" if self.pending_id is not None else latest.decision
            if latest.reference_id is not None:
                reference = self.iterations[latest.reference_id].metrics
                measured = latest.metrics
                if measured.complete and reference.complete:
                    delta = measured.duration-reference.duration
                    difference = f"{delta:+.2f} s"
                elif not measured.complete and not reference.complete:
                    delta = measured.distance-reference.distance
                    difference = f"{delta:+.2f} m"
                else:
                    difference = "volta completa" if measured.complete else "volta incompleta"
                comparison += f" · {difference.replace('.', ',')} vs #{latest.reference_id+1:02d}"
        self.comparison_indicator.set(self.confirmation_progress if self.confirmation_stop is not None else comparison)

    def set_track(self, points):
        self.points = points
        track = Track(points) if points else None
        self.bends = track_bends(track) if track else []
        self.track_length = track.length if track else 0.0
        self.drawing_stroke = False
        self.zoom, self.pan_x, self.pan_y = 1.0, 0.0, 0.0
        self.trajectory = []
        self.frames = []
        self.frame_index = 0
        self.run_sections = []
        self.weights_table.delete(*self.weights_table.get_children())
        self.profile = self.proposal = self.best = self.mapping = self.validation = None
        self.proposal_file = None
        self.best_score = None
        self.best_stop_odom_m = None
        self.official_geometry = None
        self.official_round = 1
        self.official_started = None
        self.official_attempts = []
        self.official_reference = self.live_reference = self.live_passage = None
        self.render_rules()
        self.reset_comparison()
        self.report = []
        self.session += 1
        self.number = 0
        self.session_folder = None
        self.track_loaded_at = datetime.now().astimezone()
        self.update_sidebar_visibility()
        self.sidebar_view.yview_moveto(0)
        self.render_report()
        self.draw_map()
        self.draw_track()
        self.update_buttons()

    def load_example(self):
        self.safe(lambda: self.load_track_file(DEFAULT_TRACK_PATH))

    def apply_robot_parameters(self, robot):
        if self.playback_started is not None or self.drawing:
            raise ValueError("Conclua o teste ou o desenho antes de aplicar a configuração do robô.")
        if robot == self.robot:
            self.status.set("A configuração do formulário já está em uso.")
            return
        self.robot = robot
        self.set_track(self.points)
        self.status.set("Configuração aplicada. Faça um novo mapeamento com estes parâmetros.")

    def prepare_manual_profile(self, candidate):
        if self.profile is None or self.pending_id is not None or self.playback_started is not None:
            raise ValueError("Gere um mapa ou conclua a validação pendente antes de preparar outro perfil.")
        path = self.folder() / f"iteracao-{len(self.iterations)+1:03}-proposta-manual.json"
        write_json(path, candidate)
        self.proposal = candidate
        self.proposal_file = path.relative_to(self.output).as_posix()
        self.apply_proposal()
        self.status.set("Perfil manual preparado. Clique em Validar para medir a alteração.")

    def load_pro_example(self):
        self.drawing = False
        self.set_track(pro_example())
        self.check_rules()
        self.notebook.select(self.track_panel)
        self.status.set("Exemplo Pro carregado. Faça o mapeamento e valide um perfil antes de tomar tempo.")

    def check_rules(self):
        if not self.points or self.drawing:
            raise ValueError("Carregue ou feche uma pista antes de conferir a geometria.")
        track = Track(self.points)
        findings = track_checks(track, self.robot)
        self.official_geometry = track, findings, list(curvature_markers(track))
        self.render_rules()
        self.draw_track()
        self.notebook.select(self.rules_text.master)
        return findings

    def start_official_round(self):
        if self.official_started is not None:
            self.official_round += 1
        self.official_started = None
        self.official_attempts = []
        self.official_reference = None
        if self.playback_kind == "tomada-de-tempo":
            self.live_reference = self.live_passage = None
            self.draw_track()
        self.render_rules()
        self.status.set("Nova tomada preparada. O relógio começa na primeira tentativa.")

    def render_rules(self):
        attempts = self.official_attempts
        lines = [
            f"Tomada {self.official_round} · {len(attempts)}/{MAX_ATTEMPTS} tentativas · limite de 5 minutos",
            "Vale o menor tempo entre os portões em uma volta válida. Após a chegada, o robô deve parar "
            "sozinho sobre a faixa, completamente dentro da área de 1,00 × 0,40 m, e ficar imóvel por 2 s.",
        ]
        for result in attempts:
            lap = (f"{result['tempo_volta_s']:.3f} s"
                   if result["tempo_volta_s"] is not None else "sem tempo entre portões")
            lines.append(f"Tentativa {result['tentativa']} · {result['resultado']} · {lap}")
        valid = [result["tempo_volta_s"] for result in attempts if result["resultado"] == VALID_LAP]
        if valid:
            lines.append(f"Melhor volta válida · {min(valid):.3f} s")
        lines.append("Geometria do percurso")
        if self.official_geometry:
            for ok, message in self.official_geometry[1]:
                lines.append(f"{'OK' if ok else 'Revisar'} · {message}")
        else:
            lines.append("Use Conferir geometria para avaliar a pista atual.")
        lines.append("Vistoria física · Confira manta preta, contraste, emendas de até ±2 mm, rampas "
                     "de até 5° e portões físicos com pelo menos 400 × 200 mm de vão.")
        self.rules_text.configure(state="normal")
        self.rules_text.delete("1.0", "end")
        self.rules_text.insert("1.0", "\n\n".join(lines))
        self.rules_text.configure(state="disabled")

    def start_drawing(self):
        self.drawing = True
        self.set_track([])
        self.notebook.select(self.track_panel)
        self.status.set("Arraste para desenhar. Use a rolagem para zoom. Solte junto à largada para fechar a pista.")

    def view_center(self):
        return self.canvas.winfo_width()/2, self.canvas.winfo_height()/2

    def update_sidebar_visibility(self):
        if self.drawing:
            self.sidebar_panel.pack_forget()
        elif not self.sidebar_panel.winfo_manager():
            self.sidebar_panel.pack(side="right", fill="y", before=self.track_view)

    def scroll_sidebar(self, event):
        if event.delta:
            return self.scroll_sidebar_by((-1 if event.delta > 0 else 1)
                                          * max(1, round(3*abs(event.delta)/120)))
        return "break"

    def scroll_sidebar_by(self, units):
        self.sidebar_view.yview_scroll(units, "units")
        return "break"

    def track_transform(self):
        cx, cy = self.view_center()
        scale, ox, oy = SCALE, MARGIN, MARGIN
        if self.points and not self.drawing:
            xs, ys = zip(*self.points)
            scale = min(SCALE, max(1, 2*(cx-MARGIN))/max(.01, max(xs)-min(xs)),
                        max(1, 2*(cy-MARGIN))/max(.01, max(ys)-min(ys)))
            ox, oy = cx-scale*(min(xs)+max(xs))/2, cy-scale*(min(ys)+max(ys))/2
        return (scale*self.zoom, cx+(ox-cx)*self.zoom+self.pan_x,
                cy+(oy-cy)*self.zoom+self.pan_y)

    def zoom_wheel(self, event):
        if event.delta:
            self.zoom_track(1.25 ** (event.delta/120), event.x, event.y)
        return "break"

    def zoom_track(self, factor, x=None, y=None):
        self.notebook.select(self.track_panel)
        cx, cy = self.view_center()
        x, y = (cx, cy) if x is None else (x, y)
        zoom = max(.05, min(30, self.zoom*factor))
        ratio = zoom/self.zoom
        self.pan_x = x-cx - ratio*(x-cx-self.pan_x)
        self.pan_y = y-cy - ratio*(y-cy-self.pan_y)
        self.zoom = zoom
        self.draw_track()
        return "break"

    def reset_view(self):
        self.notebook.select(self.track_panel)
        self.zoom, self.pan_x, self.pan_y = 1.0, 0.0, 0.0
        if self.drawing and self.points:
            cx, cy = self.view_center()
            xs, ys = zip(*self.points)
            scale = min(SCALE, max(1, 2*(cx-MARGIN))/max(.01, max(xs)-min(xs)),
                        max(1, 2*(cy-MARGIN))/max(.01, max(ys)-min(ys)))
            self.zoom = scale/SCALE
            self.pan_x = self.zoom*(cx-MARGIN) - scale*(min(xs)+max(xs))/2
            self.pan_y = self.zoom*(cy-MARGIN) - scale*(min(ys)+max(ys))/2
        self.draw_track()

    def begin_pan(self, event):
        self.pan_anchor = event.x, event.y
        self.canvas.configure(cursor="fleur")

    def move_pan(self, event):
        if self.pan_anchor is not None:
            x, y = self.pan_anchor
            self.pan_x += event.x-x
            self.pan_y += event.y-y
            self.pan_anchor = event.x, event.y
            self.draw_track()

    def end_pan(self, _):
        self.pan_anchor = None
        self.canvas.configure(cursor="")

    def start_stroke(self, event):
        if not self.drawing:
            return
        if self.points:
            scale, ox, oy = self.track_transform()
            x, y = self.points[-1]
            if hypot(event.x-ox-scale*x, event.y-oy-scale*y) > 18:
                self.status.set("Para continuar, arraste a partir da ponta azul do traço.")
                return
        self.drawing_stroke = True
        self.add_point(event)

    def add_point(self, event):
        if not self.drawing_stroke:
            return
        scale, ox, oy = self.track_transform()
        point = ((event.x-ox)/scale, (event.y-oy)/scale)
        if not self.points or hypot(point[0]-self.points[-1][0], point[1]-self.points[-1][1]) >= min(.012, 2/scale):
            self.points.append(point)
            self.draw_track()

    def finish_drawing(self, event):
        if not self.drawing_stroke:
            return
        self.add_point(event)
        self.drawing_stroke = False
        scale, _, _ = self.track_transform()
        close_distance = min(.25, max(.06, 14/scale))
        if len(self.points) < 4 or hypot(self.points[0][0]-self.points[-1][0],
                                        self.points[0][1]-self.points[-1][1]) > close_distance:
            self.status.set("Traço mantido. Continue pela ponta azul e solte perto da largada para fechar.")
            return
        try:
            track = Track(self.points)
        except ValueError as error:
            self.status.set(str(error))
            return
        self.drawing = False
        self.bends = track_bends(track)
        self.track_length = track.length
        self.update_sidebar_visibility()
        self.reset_view()
        self.update_buttons()
        self.status.set("Pista fechada. A marca verde indica a largada e o sentido do traço.")

    def open_track(self):
        path = filedialog.askopenfilename(filetypes=[("Pista JSON", "*.json")])
        if path:
            self.safe(lambda: self.load_track_file(path))

    def load_track_file(self, path):
        data = read_json(path)
        if not isinstance(data, dict) or not isinstance(data.get("points_m"), list):
            raise ValueError("O JSON da pista deve conter a lista points_m.")
        if any(not isinstance(p, list) or len(p) != 2
               or any(type(v) not in (int, float) or not isfinite(v) for v in p)
               for p in data["points_m"]):
            raise ValueError("Use pares de coordenadas finitas em metros.")
        track = Track(data["points_m"])
        self.drawing = False
        self.set_track(track.points)
        self.notebook.select(self.track_panel)
        self.status.set(f"Pista carregada  ·  {track.length:.2f} m")

    def save_track(self):
        if not self.points or self.drawing:
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("Pista JSON", "*.json")])
        if path:
            self.safe(lambda: write_json(path, {"points_m": self.points}))

    def draw_track(self):
        canvas = self.canvas
        canvas.delete("all")
        self.telemetry_canvas.delete("all")
        self.update_curve_counter()
        width, height = canvas.winfo_width(), canvas.winfo_height()
        scale, ox, oy = self.track_transform()
        unit = 10 ** floor(log10(45/scale))
        grid_m = next(multiplier*unit for multiplier in (1, 2, 5, 10) if multiplier*unit*scale >= 45)
        grid_px = grid_m*scale
        for x in range(round(ox % grid_px), width, round(grid_px)):
            canvas.create_line(x, 0, x, height, fill="#302929")
        for y in range(round(oy % grid_px), height, round(grid_px)):
            canvas.create_line(0, y, width, y, fill="#302929")
        canvas.create_line(20, height-22, 20+grid_px, height-22, fill=MUTED, width=2)
        scale_label = f"{grid_m*100:g} cm" if grid_m < 1 else f"{grid_m:g} m"
        canvas.create_text(27+grid_px, height-22, text=scale_label, anchor="w", fill=MUTED)
        if not self.points:
            return
        coords = lambda pts: [v for x, y in pts for v in (ox+scale*x, oy+scale*y)]
        points = self.points if self.drawing else self.points + [self.points[0]]
        if self.official_geometry:
            self.draw_official_markings(coords, zone=True)
        active = self.frames[self.frame_index].segment if self.frames else None
        if self.run_sections:
            for section, _, color, _, _ in self.run_sections:
                canvas.create_line(*coords(section), fill=color,
                                   width=max(3, LINE*scale), joinstyle="round")
                sx, sy = coords(section[:1])
                canvas.create_oval(sx-2, sy-2, sx+2, sy+2, fill=INK, outline=BG)
            section, _, color, _, _ = self.run_sections[active]
            canvas.create_line(*coords(section), fill=INK, width=13, joinstyle="round")
            canvas.create_line(*coords(section), fill=color, width=8, joinstyle="round")
        elif len(points) > 1:
            canvas.create_line(*coords(points), fill=INK, width=max(2, LINE*scale), joinstyle="round")
        if self.frame_index > 0:
            canvas.create_line(*coords(self.trajectory[:self.frame_index + 1]), fill=TRACE, width=2)
            steps = range(1, self.frame_index + 1)
            for limited, run in groupby(steps, lambda i: self.frames[i].grip_demand > 1):
                if limited:
                    start, *rest = run
                    end = rest[-1] if rest else start
                    canvas.create_line(*coords(self.trajectory[start-1:end+1]), fill=GRIP_LIMITED,
                                       width=4, capstyle="round")
        sx, sy = coords(self.points[:1])
        if self.official_geometry:
            self.draw_official_markings(coords, zone=False)
        else:
            canvas.create_oval(sx-5, sy-5, sx+5, sy+5, fill=ACCENT, outline=BG)
        if len(self.points) > 1:
            dx, dy = self.points[1][0]-self.points[0][0], self.points[1][1]-self.points[0][1]
            norm = max(1e-6, hypot(dx, dy))
            canvas.create_line(sx, sy, sx+20*dx/norm, sy+20*dy/norm, fill=ACCENT, width=2, arrow="last")
        if self.drawing:
            tx, ty = coords(self.points[-1:])
            canvas.create_oval(tx-5, ty-5, tx+5, ty+5, fill="#38bdf8", outline=BG)
        if not self.drawing:
            if self.frames:
                frame = self.frames[self.frame_index]
            else:
                (x, y), (tx, ty) = next(
                    (start, end) for start, end in zip(self.points, self.points[1:])
                    if hypot(end[0]-start[0], end[1]-start[1]) >= 1e-6)
                frame = Frame(x, y, atan2(ty-y, tx-x))
            self.draw_robot(canvas, frame, ox, oy, scale)
            self.draw_robot_detail(frame)
        if self.frames:
            legend_edge = width-15
            canvas.create_text(legend_edge, height-22, anchor="e", fill=TRACE,
                               text=f"{self.frame_index * DT:.2f} s  ·  Trajetória do robô")
            canvas.create_text(legend_edge, height-41, anchor="e", fill=GRIP_LIMITED,
                               text="Vermelho · giro limitado pela aderência")

    def update_curve_counter(self):
        total = len(self.bends)
        distance = self.frames[self.frame_index].progress_m if self.frames else 0
        current = 0
        direction = 0
        if distance > 0:
            for start, end, side in self.bends:
                if distance < start:
                    break
                current += 1
                direction = side if distance < end else 0
            if self.bends and self.bends[0][0] < 0:
                start, _, side = self.bends[0]
                if self.track_length+start <= distance < self.track_length-.001:
                    direction = side
        match direction:
            case 1:
                detail = "Curva à direita"
            case -1:
                detail = "Curva à esquerda"
            case _:
                detail = "Reta" if distance > 0 else ""
        counter = f"Curvas {current:02d}/{total:02d}"
        if self.curve_counter.get() != counter:
            self.curve_counter.set(counter)
        if self.curve_state.get() != detail:
            self.curve_state.set(detail)

    def draw_official_markings(self, coords, *, zone):
        track, _, markers = self.official_geometry
        x, y, heading = track.pose(0)

        def rectangle(x, y, heading, a, b, left, right):
            c, s = cos(heading), sin(heading)
            return coords([(x+u*c-v*s, y+u*s+v*c)
                           for u, v in ((a,left), (b,left), (b,right), (a,right))])

        if zone:
            points = rectangle(x, y, heading, -FINISH_GAP, 0, -ZONE_HALF_WIDTH, ZONE_HALF_WIDTH)
            self.canvas.create_polygon(*points, fill="#412525", outline=TRACE, dash=(4, 4))
            return
        for distance, name in ((0, "Largada"), (-FINISH_GAP, "Chegada")):
            x, y, heading = track.pose(distance)
            edge = LINE/2 + .040
            points = rectangle(x, y, heading, -LINE/2, LINE/2, edge, edge+.040)
            self.canvas.create_polygon(*points, fill=INK)
            tx, ty = coords([(x-.26*sin(heading), y+.26*cos(heading))])
            self.canvas.create_text(tx, ty, text=name, fill=ACCENT)
        for distance in markers:
            x, y, heading = track.pose(distance)
            edge = -LINE/2 - .040
            points = rectangle(x, y, heading, -LINE/2, LINE/2, edge-.040, edge)
            self.canvas.create_polygon(*points, fill=INK)

    def draw_robot_detail(self, frame):
        canvas = self.telemetry_canvas
        height = 454 if frame.segment is not None else 373
        if int(canvas.cget("height")) != height:
            canvas.configure(height=height)
        left, right = 11, DETAIL_WIDTH-12
        center = (left + right) / 2
        canvas.create_rectangle(left, 12, right, 309, fill=PANEL, outline=BORDER)
        canvas.create_text(center, 26, text="Detalhe do robô", fill=INK, font=("Segoe UI", 10, "bold"))
        silhouette_radius = max(hypot(abs(offset)+half_l, half_w)
                               for offset, half_l, half_w in self.robot.footprint_boxes)
        scale = min(680, 39/silhouette_radius)
        self.draw_robot(canvas, frame, center-scale*frame.x, 80-scale*frame.y, scale)
        canvas.create_line(left+12, 126, right-12, 126, fill=BORDER)
        canvas.create_text(left+12, 142, anchor="w", fill=MUTED, text="Velocidade")
        canvas.create_text(right-12, 142, anchor="e", fill=INK, font=("Consolas", 13, "bold"),
                           text=f"{abs(frame.velocity):.2f} m/s".replace(".", ","))
        self.draw_traction_circle(frame, center)
        self.draw_live_delta(left, right)
        if frame.segment is not None:
            _, group, color, weights, note = self.run_sections[frame.segment]
            target = f"{frame.target_speed:.2f}".replace(".", ",")
            planned_speed = frame.planned_speed if frame.planned_speed is not None else frame.target_speed
            planned = f"{planned_speed:.2f}".replace(".", ",")
            canvas.create_rectangle(left, 378, right, 452, fill=PANEL, outline=BORDER)
            canvas.create_line(left+1, 380, left+1, 450, fill=color, width=3)
            canvas.create_text(left+12, 382, anchor="nw", fill=color, font=("Segoe UI", 10, "bold"),
                               text=f"Trecho {frame.segment + 1:02d}  ·  {note or f'Grupo {group:02d}'}")
            canvas.create_text(left+12, 400, anchor="nw", fill=MUTED,
                               text=f"Nom. {weights[0]}  ·  Plan. {planned} m/s")
            canvas.create_text(left+12, 417, anchor="nw", fill=INK, text=f"Alvo aplicado  {target} m/s")
            kp = f"{frame.steering_kp:.1f}".replace(".", ",") if frame.steering_kp is not None else weights[1]
            canvas.create_text(left+12, 434, anchor="nw", fill=MUTED,
                               text=f"Kp aplicado {kp}  ·  Kd {weights[2]}")

    def draw_traction_circle(self, frame, x):
        canvas = self.telemetry_canvas
        y, radius = 225, 31
        axis = radius * 1.25
        total = hypot(frame.traction_lat, frame.traction_long)
        if total > 1:
            color, state = GRIP_LIMITED, "Acima do limite"
        elif total >= .85:
            color, state = GRIP_WARNING, "Perto do limite"
        else:
            color, state = ACCENT, "Com margem"
        drawing_scale = radius * min(1, 1.25 / max(total, 1))
        px = x + frame.traction_lat * drawing_scale
        py = y - frame.traction_long * drawing_scale
        canvas.create_text(x, 164, text="Círculo de tração", fill=INK, font=("Segoe UI", 10, "bold"))
        canvas.create_oval(x-radius, y-radius, x+radius, y+radius,
                           fill="#302626", outline="#9c7775", width=1)
        canvas.create_line(x-axis, y, x+axis, y, fill="#685351")
        canvas.create_line(x, y-axis, x, y+axis, fill="#685351")
        canvas.create_text(x, 182, text="Aceleração", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(x, 269, text="Frenagem", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(x-52, y, text="E", fill=MUTED)
        canvas.create_text(x+52, y, text="D", fill=MUTED)
        canvas.create_line(x, y, px, py, fill=color, width=2)
        canvas.create_oval(px-4, py-4, px+4, py+4, fill=color, outline=BG)
        percent = ">100" if 1 < total < 1.005 else str(round(100*total))
        canvas.create_text(x, 291, text=f"{percent}%  ·  {state}", fill=color,
                           font=("Segoe UI", 10, "bold"))

    def draw_live_delta(self, left, right):
        canvas = self.telemetry_canvas
        label, value, color = self.live_delta()
        canvas.create_rectangle(left, 316, right, 371, fill="#302020", outline=BRAND)
        canvas.create_line(left+1, 318, left+1, 369, fill=BRAND_BRIGHT, width=3)
        canvas.create_text(left+12, 324, text="Delta ao vivo", anchor="nw", fill=INK,
                           font=("Segoe UI", 9, "bold"))
        if label:
            canvas.create_text(right-11, 324, text=f"Ref. {label}", anchor="ne", fill=MUTED,
                               font=("Segoe UI", 9))
        numeric = value.endswith(" s")
        canvas.create_text(left+12, 341 if numeric else 346, text=value, anchor="nw", fill=color,
                           font=("Consolas", 16, "bold") if numeric else ("Segoe UI", 10))

    def draw_robot(self, canvas, frame, ox, oy, scale):
        c, s = cos(frame.heading), sin(frame.heading)

        def screen(forward, lateral):
            return (ox + scale * (frame.x + c*forward - s*lateral),
                    oy + scale * (frame.y + s*forward + c*lateral))

        def part(back, front, left, right, color, outline):
            canvas.create_polygon(screen(back, left), screen(front, left),
                                  screen(front, right), screen(back, right),
                                  fill=color, outline=outline)

        half_width = self.robot.wheelbase / 2
        wheel_radius = self.robot.diameter / 2
        wheel_half_width = self.robot.wheel_width / 2
        sensor_edge = self.robot.sensor_board_width / 2
        body_back, body_front = self.robot.body_edges
        shoulder_front = body_back + self.robot.body_length*.45
        center_half_width = self.robot.body_width*.26
        sensor_forward = self.robot.sensor_forward
        board_back, board_front = self.robot.sensor_board_edges
        part(body_back, shoulder_front, -half_width, half_width, "#347ecc", "#7cb9f1")
        part(shoulder_front, body_front, -center_half_width, center_half_width, "#347ecc", "#7cb9f1")
        part(body_front, sensor_forward, -.007, .007, "#347ecc", "#7cb9f1")
        for lateral in (-half_width, half_width):
            side = 1 if lateral > 0 else -1
            motor_left, motor_right = sorted((lateral-side*.021, lateral-side*wheel_half_width))
            part(-.009, .009, motor_left, motor_right, "#9cabb8", "#475669")
            part(-wheel_radius, wheel_radius, lateral-wheel_half_width, lateral+wheel_half_width,
                 "#ffcc27", "#ba790e")
            highlight = lateral-.6*wheel_half_width
            canvas.create_line(*screen(-wheel_radius+.004, highlight),
                               *screen(wheel_radius-.004, highlight), fill="#ffe581")
        part(body_back+.007, body_front-.007, -.028, .028, "#b9934c", "#dec37a")
        part(-.006, .026, -.021, -.004, "#252b36", "#657385")
        part(-.004, .028, .009, .025, "#202832", "#657385")
        part(-.004, .007, .011, .023, "#a6b5c0", "#637486")
        part(board_back, board_front, -sensor_edge, sensor_edge,
             "#d9343d", "#ff8075")
        for i, color in enumerate(("#ef454c", "#f08a31", "#ffcd44", "#46bc76", "#34a7ef", "#a587e6")):
            offset = (i-2.5)*.0018
            canvas.create_line(screen(-.009, .010+offset), screen(.012, .006+offset),
                               screen(body_front+self.robot.sensor_gap*.45, offset),
                               screen(board_back+.001, offset),
                               fill=color, width=max(1, min(2, scale*.0016)), smooth=True)
        radius = max(1.3, min(2.9, scale * .003))
        for index, lateral in enumerate(self.robot.sensor_offsets):
            sx, sy = screen(sensor_forward, lateral)
            bright = int(42 + 207 * frame.readings[index]) if frame.readings is not None else 42
            canvas.create_oval(sx-radius, sy-radius, sx+radius, sy+radius,
                               fill=f"#32{bright:02x}74", outline="#49151f")

    def change_playback_rate(self):
        if self.playback_started is None or self.playback_kind == "tomada-de-tempo":
            return
        now = monotonic()
        self.playback_offset += (now-self.playback_started)*self.playback_rate
        self.playback_started = now
        self.playback_rate = self.playback_preference.get()

    def animate_run(self):
        self.playback_job = None
        elapsed = self.playback_offset + (monotonic()-self.playback_started)*self.playback_rate
        self.frame_index = min(len(self.frames) - 1, int(elapsed / DT))
        self.draw_track()
        if self.frame_index == len(self.frames) - 1:
            self.playback_started = None
            self.status.set(self.run_summary)
            self.update_buttons()
        else:
            self.playback_job = self.root.after(16, self.animate_run)

    def draw_map(self):
        canvas = self.map_canvas
        canvas.delete("all")
        if self.profile is None:
            self.map_notes_count.set("")
            canvas.create_text(22, 22, text="Gere o mapa a partir da volta de mapeamento.", anchor="nw", fill=MUTED)
            return
        points = self.profile["path"]
        xs, ys = zip(*points)
        w, h = canvas.winfo_width(), canvas.winfo_height()
        scale = min(max(1, w-72)/max(.01, max(xs)-min(xs)), max(1, h-100)/max(.01, max(ys)-min(ys)))
        center_x, center_y = (max(xs)+min(xs))/2, (max(ys)+min(ys))/2

        def project(point):
            x, y = point
            return w/2 + (x-center_x)*scale, h/2 + (y-center_y)*scale

        sections = self.profile.get("path_sections")
        if sections is None:
            sections = list(zip(points, points[1:]))
        technical = self.map_technical.get()
        notes = map_note_groups(self.profile, technical=technical)
        count = f"{len(sections)} setores de controle"
        self.map_notes_count.set(count if technical else f"{len(notes)} notas agrupadas · {count}")
        for first, end, note in notes:
            color = CURVE_COLORS[note[-1] if len(note) == 2 else note] if note else ACCENT
            screen = [project(point) for index in range(first, end)
                      for point in (sections[index] if index == first else sections[index][1:])]
            canvas.create_line(*screen, fill=color, width=4, joinstyle="round")
            middle = (len(screen)-1)//2
            x, y = screen[middle]
            tx, ty = screen[middle+1]
            canvas.create_text((x+tx)/2, (y+ty)/2-11, text=note or str(first+1), fill=color)
        canvas.create_text(18, 17, anchor="nw", fill=MUTED,
                           text=f"Odometria  ·  {self.profile['length']:.2f} m"
                                "    |    D = direita, E = esquerda    |    1 fechada → 5 aberta")
        for point, label in ((points[0], "I"), (points[-1], "F")):
            x, y = project(point)
            canvas.create_oval(x-6, y-6, x+6, y+6, fill=INK)
            canvas.create_text(x, y+15, text=label, fill=INK)

    def show_curve_evidence(self, _event=None):
        selection = self.curve_table.selection()
        if not selection:
            self.curve_evidence.set("")
            return
        row = self.curve_report[int(selection[0])]
        if not row["setores"]:
            self.curve_evidence.set("Sem amostras nesta região.")
            return
        signals = [f"QTR na borda {row['borda_pct']:.0f}% do tempo", f"sem linha {row['perda_qtr_pct']:.1f}%",
                   f"no alvo {row['no_alvo_pct']:.0f}%"]
        if row["tracao_p95_pct"] is not None:
            signals.append(f"tração combinada P95 {row['tracao_p95_pct']:.0f}%")
        context = []
        if row["curva"]:
            if row["episodios_perda_qtr"] and row["erro_pre_perda_s"]:
                context.append(f"erro no sentido da curva por {1000*row['erro_pre_perda_s']:.0f} ms antes da perda")
            elif row["erro_persistente_s"]:
                context.append(f"erro sustentado no sentido da curva {1000*row['erro_persistente_s']:.0f} ms")
            if row["lateral_max_pct"] is not None:
                context.append(f"pico lateral {row['lateral_max_pct']:.0f}%")
            if "alinhamento" in row["ajuste"].lower():
                context.append(f"sentido do mapa coincide em {row['mapa_coerente_pct']:.0f}% da curva")
        else:
            signals.append(f"inversões do QTR {row['inversoes_qtr']}")
        if row["episodios_perda_qtr"]:
            count = row["episodios_perda_qtr"]
            signals.append(f"{count} {'perda de linha' if count == 1 else 'perdas de linha'}")
            if row["tracao_pre_perda_max_pct"] is not None:
                context.append(f"pico de tração antes da perda {row['tracao_pre_perda_max_pct']:.0f}%")
        if row["frenagem_recuperacao_s"]:
            context.append(f"frenagem na recuperação {1000*row['frenagem_recuperacao_s']:.0f} ms")
        self.curve_evidence.set("\n".join(" · ".join(line) for line in (signals, context) if line).replace(".", ","))

    def render_report(self):
        self.curve_table.delete(*self.curve_table.get_children())
        self.curve_evidence.set("")
        if not self.report:
            self.curve_report = []
            self.diagnostic_context.set("Valide uma passada e clique em Diagnosticar.")
        focus = "0"
        for index, row in enumerate(self.curve_report):
            duration, delta, speed = (row[key] for key in ("tempo_s", "delta_s", "vel_saida_m_s"))
            selected = row["selecionado"]
            adjustment = row["proximo_ajuste"] if selected else row["ajuste"]
            if selected:
                focus = str(index)
            self.curve_table.insert("", "end", iid=str(index), tags=("proposta",) if selected else (), values=(
                row["regiao"], "—" if duration is None else f"{duration:.3f}".replace(".", ","),
                "—" if delta is None else f"{delta:+.3f}".replace(".", ","),
                "—" if speed is None else f"{speed:.2f}".replace(".", ","),
                row["limitante"], ("▶ " if selected else "")+adjustment))
        if self.curve_report:
            self.curve_table.selection_set(focus)
            self.curve_table.see(focus)
            self.show_curve_evidence()
        self.table.delete(*self.table.get_children())
        for row in self.report:
            delta = row["saldo_s"]
            self.table.insert("", "end", values=(
                f"{row['trecho']} {row['nota']}".strip(),
                f"{1000*row['erro_rms_m']:.1f}" if row["tempo_s"] else "—",
                *(f"{100*row[key]:.0f}" if row["tempo_s"] else "—"
                  for key in ("line_edge", "line_lost", "body_off_line", "motor_sat")),
                f"{row['pico_lateral_m_s2']:.2f}" if row["tempo_s"] else "—",
                f"{delta:+.2f}" if delta is not None else "—",
                f"{row['v_proposta_m_s']:.2f}", row["acao"]))

    def folder(self):
        folder = self.session_folder
        while folder is None:
            candidate = self.output / f"sessao-{self.session:03}"
            try:
                candidate.mkdir(parents=True)
            except FileExistsError:
                self.session += 1
            else:
                self.session_folder = folder = candidate
        write_json(folder / "pista.json", {"points_m": self.points})
        write_json(folder / "robo.json", asdict(self.robot))
        return folder

    def export_session(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".zip", filetypes=[("Sessão do simulador", "*.zip")],
            initialfile=f"sessao-simulador-{self.track_loaded_at:%Y%m%d-%H%M%S}-{self.session:03}.zip")
        if path:
            self.safe(lambda: self.write_session_archive(Path(path)))

    def write_session_archive(self, destination):
        if self.session_folder is None:
            raise ValueError("Rode um teste na pista atual para exportar a sessão.")
        files = [file for file in sorted(self.session_folder.iterdir())
                 if file.is_file() and file.suffix in (".csv", ".json")]
        iterations = []
        for index, iteration in enumerate(self.iterations, 1):
            if iteration.session != self.session:
                continue
            reference_id = iteration.reference_id
            reference = self.iterations[reference_id].metrics if reference_id is not None else None
            current = iteration.metrics
            delta = (round(current.duration-reference.duration, 3)
                     if current and reference and current.complete and reference.complete else None)
            iterations.append({
                "iteracao": index,
                "sessao": f"sessao-{iteration.session:03}",
                "referencia": reference_id+1 if reference_id is not None else None,
                "decisao": iteration.decision,
                "validacao_automatica": iteration.automatic,
                "delta_tempo_s": delta,
                "confirmacao_pareada": iteration.confirmation,
                "metricas": ({"volta_completa": current.complete, "tempo_s": current.duration,
                              "distancia_encoder_m": current.distance,
                              "fracao_sem_linha": current.lost_fraction,
                              "fracao_corpo_fora_da_faixa": current.body_off_fraction}
                             if current else None),
                "telemetria_csv": iteration.validation_csv,
                "diagnostico_csv": iteration.diagnostic_csv,
                "diagnostico_curvas_csv": iteration.curve_csv,
                "proposta_aplicada_json": iteration.applied_proposal_json,
            })
        manifest = {
            "versao_exportacao": 1,
            "aberto_em": self.opened_at.isoformat(),
            "pista_carregada_em": self.track_loaded_at.isoformat(),
            "sessao": self.session_folder.name,
            "exportado_em": datetime.now().astimezone().isoformat(),
            "arquivos": [file.relative_to(self.output).as_posix() for file in files],
            "iteracoes": iterations,
        }
        with NamedTemporaryFile(dir=destination.parent, suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
        try:
            with ZipFile(temporary, "w", compression=ZIP_DEFLATED) as archive:
                archive.writestr("resumo-da-sessao.json", json.dumps(manifest, ensure_ascii=False, indent=2))
                for file in files:
                    archive.write(file, file.relative_to(self.output).as_posix())
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
        self.status.set(f"Sessão exportada  ·  {len(files)} arquivos de dados  ·  {destination.name}")

    def reset_comparison(self):
        self.optimizer.cancel()
        self.map_history_start = len(self.iterations)
        self.cancel_confirmation()
        if self.pending_id is not None:
            self.iterations[self.pending_id].decision = "Teste cancelado"
            self.render_iteration(self.pending_id)
        self.reference_id = self.pending_id = self.last_iteration_id = None
        self.reference_rows = None
        self.latest_validation_id = None
        self.live_reference = self.live_passage = None

    def cancel_confirmation(self):
        if self.confirmation_stop is not None:
            self.confirmation_stop.set()
            self.confirmation_stop = None
        if self.confirmation_job is not None:
            self.root.after_cancel(self.confirmation_job)
            self.confirmation_job = None
        self.confirmation_progress = ""

    def stop_confirmation_on_close(self, event):
        if event.widget is self.root:
            self.optimizer.cancel()
            self.cancel_confirmation()

    def live_delta(self):
        if not self.live_passage or not self.live_reference:
            return "", "Sem referência", MUTED
        label, reference = self.live_reference
        elapsed = self.frame_index * DT
        if elapsed < self.live_start:
            return label, "Antes da largada", MUTED
        if self.live_finish is not None and elapsed >= self.live_finish:
            elapsed = self.live_finish
            distance = self.live_passage.distances[-1]
        else:
            distance = max(0, min(self.frames[self.frame_index].progress_m,
                                  self.live_passage.distances[-1]))
        comparison = reference.time_at(distance)
        if comparison is None:
            return label, "Além da referência", MUTED
        delta = elapsed-self.live_start-comparison
        if abs(delta) < .0005:
            delta = 0.0
        value = f"{delta:+.3f} s".replace(".", ",")
        color = GRIP_LIMITED if delta > 0 else ACCENT if delta < 0 else MUTED
        return label, value, color

    def render_iteration(self, index):
        iteration = self.iterations[index]
        metrics = iteration.metrics
        reference_id = iteration.reference_id
        reference = self.iterations[reference_id].metrics if reference_id is not None else None
        delta = (f"{metrics.duration-reference.duration:+.2f}"
                 if metrics and reference and metrics.complete and reference.complete else "—")
        summary = "—"
        if iteration.confirmation:
            result = iteration.confirmation
            median_s = result["delta_mediano_s"]
            delta_text = f"{median_s:+.3f} s" if median_s is not None else "sem par válido"
            summary = f"{result['validas']}/{len(PAIR_SEEDS)} · {delta_text.replace('.', ',')}"
        values = (index+1, iteration.session, reference_id+1 if reference_id is not None else "—",
                  ("Completa" if metrics.complete else "Incompleta") if metrics else "—",
                  f"{metrics.duration:.2f}" if metrics else "—", delta,
                  f"{metrics.distance:.2f}" if metrics else "—",
                  f"{100*metrics.lost_fraction:.1f}" if metrics else "—",
                  f"{100*metrics.body_off_fraction:.2f}" if metrics else "—",
                  "Gerado" if iteration.diagnosed else "—", summary, iteration.decision)
        tags = {"Aceita": ("accepted",), "Rejeitada": ("rejected",)}.get(iteration.decision, ())
        key = str(index)
        if self.history_table.exists(key):
            self.history_table.item(key, values=values, tags=tags)
        else:
            self.history_table.insert("", "end", iid=key, values=values, tags=tags)
        self.history_table.see(key)
        if key in self.history_table.selection():
            self.show_pair_details()

    def show_pair_details(self):
        selection = self.history_table.selection()
        iteration = self.iterations[int(selection[0])] if selection else None
        result = iteration.confirmation if iteration else None
        details = ["Validação automática"] if iteration and iteration.automatic else []
        if result:
            for pair in result["pares"]:
                times = []
                for profile in ("referencia", "candidata"):
                    seconds = pair[f"{profile}_tempo_s"]
                    time_text = f"{seconds:.2f} s".replace(".", ",") if pair[f"{profile}_valida"] else "inválida"
                    times.append(time_text)
                details.append(f"Semente {pair['semente']} · {times[0]} → {times[1]}")
        self.pair_details.set("     ".join(details))

    def prepare_sections(self, track, profile):
        self.run_sections = []
        if profile is None:
            return
        self.weights_table.delete(*self.weights_table.get_children())
        groups = {}
        bounds = [0, *profile["ends"], profile["length"]]
        for index, segment in enumerate(profile["segments"]):
            weights = display_weights(segment)
            note = segment.get("label", "")
            if weights not in groups:
                group = len(groups) + 1
                rgb = hsv_to_rgb((group * .618034) % 1, .41, .87)
                color = "#" + "".join(f"{round(channel*255):02x}" for channel in rgb)
                groups[weights] = group, color, []
            group, color, members = groups[weights]
            members.append(f"{index+1:02d} {note}".strip())
            start, end = (bound * track.length / profile["length"] for bound in bounds[index:index+2])
            section = track_section(track, start, end)
            self.run_sections.append((section, group, color, weights, note))
        for weights, (group, color, members) in groups.items():
            tag = f"grupo-{group}"
            self.weights_table.tag_configure(tag, foreground=color)
            self.weights_table.insert("", "end", tags=(tag,),
                                      values=(f"{group:02d}", ", ".join(members), *weights))

    def run(self, name, profile=None, seed=7, *, trial=None):
        result = simulate_pass(self.points, self.robot, profile, seed, trial)
        return self.record_run(result, name, profile, seed, trial=trial)

    def record_run(self, result, name, profile, seed, *, trial=None, automatic=False):
        track, frames, rows, trajectory, reason = result
        self.live_reference = self.live_passage = None
        self.live_start = self.live_finish = None
        if trial and trial.start_s is not None:
            self.live_start, self.live_finish = trial.start_s, trial.finish_s
            self.live_passage = Passage.from_frames(frames, trial.start_s, trial.finish_s,
                                                    track.length-FINISH_GAP)
            self.live_reference = self.official_reference
        elif name == "validacao":
            self.live_start = 0.0
            self.live_passage = Passage.from_frames(frames, 0, None, track.length-.001)
            if self.reference_id is not None:
                reference = self.iterations[self.reference_id].passage
                if reference:
                    self.live_reference = f"#{self.reference_id+1:02d}", reference
        folder = self.folder()
        self.number += 1
        prefix = folder / f"{self.number:02}-{name}"
        write_csv(prefix.with_suffix(".csv"), rows)
        metadata = {"seed": seed, "resultado": reason, "tempo_s": rows[-1]["t_s"], "perfil": profile}
        if name == "validacao":
            metadata["validacao_automatica"] = automatic
        if trial:
            result = {"tomada": self.official_round, "tentativa": len(self.official_attempts)+1,
                      "resultado": reason, "tempo_volta_s": trial.lap_s,
                      "duracao_tentativa_s": rows[-1]["t_s"], "largada_s": trial.start_s,
                      "chegada_s": trial.finish_s, "tempo_imovel_s": trial.stopped_for_s,
                      "orcamento_ao_iniciar_s": trial.budget,
                      "parada_calibrada": trial.stop_odom_m is not None,
                      "alvo_parada_encoder_m": trial.stop_distance(profile["length"]),
                      "posicao_final_relativa_chegada_m": frames[-1].progress_m-(track.length-FINISH_GAP)}
            metadata["robocore_pro"] = result
        write_json(prefix.with_suffix(".json"), metadata)
        if trial:
            self.official_attempts.append(result)
            if (reason == VALID_LAP and (self.official_reference is None
                    or trial.lap_s < self.official_reference[1].times[-1])):
                self.official_reference = f"T{result['tentativa']}", self.live_passage
            self.render_rules()
        self.trajectory = trajectory
        self.frames = frames
        self.playback_kind = name
        self.frame_index = len(frames)-1 if automatic else 0
        self.prepare_sections(track, profile)
        if not automatic:
            self.notebook.select(self.track_panel)
        self.draw_track()
        label = name.replace("-", " ").capitalize()
        self.run_summary = f"{label}  ·  {reason}  ·  {rows[-1]['t_s']:.2f} s  ·  {folder.name}"
        if automatic:
            return rows, reason, prefix
        self.status.set(f"{label} em andamento  ·  o array brilha onde detecta a linha")
        if self.playback_job is not None:
            self.root.after_cancel(self.playback_job)
        self.playback_offset = 0.0
        self.playback_rate = 1 if trial else self.playback_preference.get()
        self.playback_started = monotonic()
        self.update_buttons()
        self.playback_job = self.root.after(16, self.animate_run)
        return rows, reason, prefix

    def run_mapping(self):
        self.mapping = None
        rows, reason, _ = self.run("mapeamento")
        if reason == "volta completa":
            self.mapping = rows

    def generate_map(self):
        profile = generate(self.mapping, self.robot)
        write_json(self.folder() / f"{self.number:02}-mapa.json", profile)
        self.profile = profile
        self.reset_comparison()
        self.draw_track()
        self.validation = self.proposal = None
        self.proposal_file = None
        self.report = []
        self.render_report()
        self.notebook.select(self.map_panel)
        self.draw_map()
        self.status.set(f"Mapa gerado  ·  {len(profile['segments'])} trechos  ·  pronto para validar")

    def run_validation(self):
        tested_profile = self.profile
        rows, reason, prefix = self.run("validacao", tested_profile, seed=SCREENING_SEED)
        self.record_validation(tested_profile, rows, reason, prefix)

    def start_auto_validation(self):
        future = Future()
        points, robot, profile = self.points, self.robot, self.profile

        def compute():
            try:
                future.set_result(simulate_pass(points, robot, profile, SCREENING_SEED))
            except Exception as error:
                future.set_exception(error)

        Thread(target=compute, daemon=True).start()
        return future

    def accept_auto_validation(self, result):
        tested_profile = self.profile
        rows, reason, prefix = self.record_run(result, "validacao", tested_profile, SCREENING_SEED, automatic=True)
        self.record_validation(tested_profile, rows, reason, prefix, automatic=True)

    def record_validation(self, tested_profile, rows, reason, prefix, *, automatic=False):
        self.validation = rows, reason, prefix, tested_profile
        self.proposal = None
        self.proposal_file = None
        self.report = []
        self.render_report()
        metrics = ValidationMetrics.from_run(rows, reason, self.robot)
        if metrics.body_off_fraction:
            self.run_summary += "  ·  corpo saiu da faixa"
        index = self.pending_id
        if index is None:
            index = len(self.iterations)
            self.iterations.append(Iteration(self.session, tested_profile, self.reference_id))
        iteration = self.iterations[index]
        iteration.automatic = automatic
        iteration.metrics = metrics
        iteration.seed_metrics[SCREENING_SEED] = metrics
        iteration.passage = self.live_passage
        iteration.validation_csv = prefix.with_suffix(".csv").relative_to(self.output).as_posix()
        self.last_iteration_id = self.latest_validation_id = index
        reference_id = iteration.reference_id
        if self.pending_id is not None and reference_id is not None:
            reference = self.iterations[reference_id]
            if reference.metrics.competitive and metrics.competitive and metrics.duration < reference.metrics.duration:
                self.start_confirmation(index, reference)
                return
            improved = not reference.metrics.competitive and metrics.rank < reference.metrics.rank
            decision = "Aceita" if improved else "Rejeitada"
        else:
            decision = "Referência inicial" if reference_id is None else "Repetição"
        self.finish_validation(index, decision)

    def start_confirmation(self, index, reference):
        iteration = self.iterations[index]
        iteration.decision = "Confirmando"
        self.render_iteration(index)
        events = Queue()
        self.confirmation_progress = "Confirmando 3 pares"
        self.confirmation_stop = Event()
        track = Track(self.points)
        worker = Thread(target=run_pairs, daemon=True, args=(
            track, self.robot, reference.profile, iteration.profile,
            reference.seed_metrics.copy(), iteration.seed_metrics.copy(), events, self.confirmation_stop))
        worker.start()
        self.confirmation_job = self.root.after(100, self.poll_confirmation, index, events)
        self.update_buttons()

    def poll_confirmation(self, index, events):
        self.confirmation_job = None
        while True:
            try:
                kind, payload = events.get_nowait()
            except Empty:
                self.confirmation_job = self.root.after(100, self.poll_confirmation, index, events)
                return
            if kind == "progress":
                self.confirmation_progress = payload
                self.update_iteration_indicator()
                continue
            self.confirmation_stop = None
            iteration = self.iterations[index]
            if kind == "done":
                result, previous, current = payload
                self.iterations[iteration.reference_id].seed_metrics.update(previous)
                iteration.seed_metrics.update(current)
                iteration.confirmation = result
                self.finish_validation(index, "Aceita" if result["aceita"] else "Rejeitada")
            else:
                iteration.decision = "Confirmação interrompida"
                self.validation = None
                self.run_summary += f" · falha na confirmação ({payload}). Clique em Validar para repetir."
                self.render_iteration(index)
            if self.playback_started is None:
                self.status.set(self.run_summary)
            self.update_buttons()
            return

    def finish_validation(self, index, decision):
        iteration = self.iterations[index]
        iteration.decision = decision
        metrics = iteration.metrics
        rows = self.validation[0]
        self.pending_id = None
        if decision in ("Referência inicial", "Aceita"):
            self.reference_id = index
            self.reference_rows = rows
            if decision == "Aceita":
                self.run_summary += "  ·  proposta aceita"
            if metrics.complete and (self.best_score is None or metrics.rank < self.best_score):
                write_json(self.folder() / "melhor-perfil.json", iteration.profile)
                self.best, self.best_score = iteration.profile, metrics.rank
                self.best_stop_odom_m = calibrate_stop_odom(rows, self.frames, self.robot, self.track_length)
                self.run_summary += "  ·  melhor perfil atualizado"
        elif decision == "Rejeitada":
            self.profile = self.iterations[iteration.reference_id].profile
            self.run_summary += "  ·  proposta rejeitada, perfil anterior restaurado"
        self.render_iteration(index)

    def run_diagnosis(self):
        rows, reason, prefix, tested_profile = self.validation
        last_id = self.last_iteration_id
        last = self.iterations[last_id] if last_id is not None else None
        reference_rows = None
        if last and last.reference_id is not None:
            previous = self.iterations[last.reference_id]
            if last.reference_id == self.reference_id:
                reference_rows = self.reference_rows
            elif previous.validation_csv:
                reference_rows = read_csv(self.output / previous.validation_csv)
        trials = [(self.iterations[it.reference_id].profile, it.profile, it.decision == "Aceita")
                  for it in self.iterations[self.map_history_start:]
                  if it.reference_id is not None and it.decision in ("Aceita", "Rejeitada")]
        proposal, report, curves = diagnose_pass(rows, tested_profile, self.robot, reason == "volta completa",
                                                self.bends, self.track_length, reference_rows=reference_rows,
                                                trials=trials)
        next_curves = curves
        base_id = last_id
        base_profile = tested_profile
        based_on_reference = (last and last.decision == "Rejeitada" and self.reference_id is not None
                              and self.iterations[self.reference_id].metrics.competitive)
        if based_on_reference:
            base_id = self.reference_id
            reference = self.iterations[base_id]
            base_profile = reference.profile
            proposal, _, next_curves = diagnose_pass(
                self.reference_rows, reference.profile, self.robot, reference.metrics.complete,
                self.bends, self.track_length, trials=trials)
            action = next((row["ajuste"] for row in next_curves if row["selecionado"]), "manter")
            describe_proposal(report, tested_profile, reference.profile, proposal, action)
        for row, planned in zip(curves, next_curves):
            row["proximo_ajuste"] = planned["ajuste"] if planned["selecionado"] else ""
            if planned["selecionado"]:
                key = planned["parametro"]
                values = {(before.get(key, 0), after[key]) for before, after
                          in zip(base_profile["segments"], proposal["segments"]) if before != after}
                if len(values) == 1:
                    before, after = values.pop()
                    row["proximo_ajuste"] += f" ({before:.2f} → {after:.2f})".replace(".", ",")
                elif planned["passo_pct"] is not None:
                    row["proximo_ajuste"] += f" ({planned['passo_pct']:+.1f}%)".replace(".", ",")
            row["base_da_proposta"] = base_id+1 if base_id is not None else None
            row["referencia_do_delta"] = last.reference_id+1 if last and last.reference_id is not None else None
            for key in ("selecionado", "setores_propostos", "parametro", "passo_pct"):
                row[key] = planned[key]
        diagnostic_file = prefix.with_name(prefix.name + "-diagnostico.csv")
        curve_file = prefix.with_name(prefix.name + "-curvas.csv")
        proposal_file = prefix.with_name(prefix.name + "-proposta.json")
        write_csv(diagnostic_file, report)
        write_csv(curve_file, curves)
        self.report, self.curve_report = report, curves
        self.proposal = self.proposal_file = None
        if proposal != self.profile:
            write_json(proposal_file, proposal)
            self.proposal = proposal
            self.proposal_file = proposal_file.relative_to(self.output).as_posix()
        if self.last_iteration_id is not None:
            self.iterations[self.last_iteration_id].diagnosed = True
            self.iterations[self.last_iteration_id].diagnostic_csv = diagnostic_file.relative_to(self.output).as_posix()
            self.iterations[self.last_iteration_id].curve_csv = curve_file.relative_to(self.output).as_posix()
            self.render_iteration(self.last_iteration_id)
        self.render_report()
        comparison = (f"Δ local vs. #{last.reference_id+1:02d}" if reference_rows is not None
                      else "Primeira medição" if last else "Tomada oficial")
        context = f"{comparison} · parcial da entrada até 0,50 m após a saída ou até a curva seguinte."
        chosen = next((row for row in curves if row["selecionado"]), None)
        if chosen:
            origin = f" sobre #{base_id+1:02d}" if base_id is not None else ""
            context += f"  Próximo teste{origin} · {chosen['regiao']} · {chosen['proximo_ajuste']}."
        elif proposal != self.profile:
            context += "  Ajustes de recuperação na aba Setores técnicos."
        self.diagnostic_context.set(context)
        if not self.optimizer.active:
            self.notebook.select(2)
            self.diagnostic_views.select(0)
        if self.proposal is None:
            self.status.set("Diagnóstico salvo. A busca não encontrou um novo ajuste com margem nesta passagem.")
        elif based_on_reference:
            self.status.set("Proposta salva a partir da referência. Aplique para medir o novo ajuste.")
        else:
            self.status.set("Proposta salva. Aplique para testar outro perfil ou tome tempo com o melhor já validado.")

    def apply_proposal(self):
        self.profile, self.proposal = self.proposal, None
        self.pending_id = len(self.iterations)
        self.iterations.append(Iteration(self.session, self.profile, self.reference_id,
                                         applied_proposal_json=self.proposal_file))
        self.proposal_file = None
        self.render_iteration(self.pending_id)
        self.validation = None
        self.status.set("Proposta aplicada. Rode outra validação para medir o efeito.")

    def run_timed(self):
        if len(self.official_attempts) >= MAX_ATTEMPTS:
            raise ValueError("As três tentativas foram usadas. Abra RoboCore Pro para iniciar outra tomada.")
        findings = self.check_rules()
        if not all(ok for ok, _ in findings):
            raise ValueError("Revise os itens sinalizados na aba RoboCore Pro antes da tentativa oficial.")
        if self.official_started is None:
            self.official_started = monotonic()
        used = max(monotonic()-self.official_started,
                   sum(result["duracao_tentativa_s"] for result in self.official_attempts))
        budget = SESSION_SECONDS-used
        if budget < DT:
            raise ValueError("Os 5 minutos terminaram. Inicie outra tomada na aba RoboCore Pro.")
        trial = OfficialTrial(budget, stop_odom_m=self.best_stop_odom_m)
        seed = 19 + len(self.official_attempts)
        rows, reason, prefix = self.run("tomada-de-tempo", self.best, seed=seed, trial=trial)
        if trial.start_s is not None:
            start = next((i for i, row in enumerate(rows) if row["t_s"] > trial.start_s), len(rows))
            initial = rows[start-1] if start else {"enc_left": 0, "enc_right": 0}
            timed_rows = [dict(row, t_s=row["t_s"]-trial.start_s,
                               enc_left=row["enc_left"]-initial["enc_left"],
                               enc_right=row["enc_right"]-initial["enc_right"])
                          for row in rows[start:]
                          if trial.finish_s is None or row["t_s"] <= trial.finish_s]
            if timed_rows:
                self.validation = timed_rows, "trecho cronometrado", prefix, self.best
                self.last_iteration_id = None
                self.proposal = self.proposal_file = None
                self.report = []
                self.render_report()
        stopwatch = f" · tempo entre portões {trial.lap_s:.3f} s" if trial.lap_s is not None else ""
        self.run_summary = (f"Pro · tentativa {len(self.official_attempts)}/{MAX_ATTEMPTS} · {reason}"
                            f"{stopwatch}")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()

