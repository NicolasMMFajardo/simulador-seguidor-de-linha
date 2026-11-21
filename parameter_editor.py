"""Formulários de configuração do robô e de ajuste do perfil de velocidade."""
import tkinter as tk
from copy import deepcopy
from dataclasses import asdict, dataclass
from math import isfinite
from tkinter import filedialog, ttk

from offboard import read_json, write_json
from simulator import CURVE_LEVELS, Robot


@dataclass(frozen=True)
class Field:
    key: str
    label: str
    scale: float = 1
    minimum: float | None = 0
    strictly_positive: bool = True
    integer: bool = False
    optional: bool = False
    maximum: float | None = None

    def display(self, value):
        return "" if value is None else f"{value * self.scale:.12g}".replace(".", ",")

    def parse(self, raw):
        raw = raw.strip().replace(",", ".")
        if not raw and self.optional:
            return None
        try:
            value = float(raw)/self.scale
        except ValueError:
            raise ValueError(f"Informe um número para {self.label}.") from None
        if not isfinite(value):
            raise ValueError(f"Informe um número finito para {self.label}.")
        if self.minimum is not None:
            if value < self.minimum or (self.strictly_positive and value == self.minimum):
                operator = "maior que" if self.strictly_positive else "maior ou igual a"
                raise ValueError(f"{self.label} deve ser {operator} {self.display(self.minimum)}.")
        if self.maximum is not None and value > self.maximum:
            raise ValueError(f"{self.label} deve ser menor ou igual a {self.display(self.maximum)}.")
        if self.integer and not value.is_integer():
            raise ValueError(f"{self.label} deve ser um número inteiro.")
        return int(value) if self.integer else value


SUCTION_FIELDS = (
    Field("suction_force_n", "Força de sucção (N)", strictly_positive=False, optional=True),
    Field("mass_kg", "Massa do robô (g)", 1000, optional=True),
)

ROBOT_GROUPS = (
    ("Geometria", (
        Field("wheelbase", "Bitola centro a centro (mm)", 1000),
        Field("diameter", "Diâmetro da roda (mm)", 1000),
        Field("wheel_width", "Largura do pneu (mm)", 1000),
        Field("body_length", "Profundidade do chassi (mm)", 1000),
        Field("body_width", "Largura do chassi (mm)", 1000),
        Field("body_height", "Altura do robô (mm)", 1000),
        Field("axle_from_rear", "Eixo a partir da traseira (mm)", 1000, strictly_positive=False),
    )),
    ("Motores e odometria", (
        Field("motor_rpm", "Rotação máxima de saída (rpm)"),
        Field("ticks_per_turn", "Contagens por volta da roda", minimum=1, strictly_positive=False, integer=True),
        Field("motor_tau", "Constante do motor (ms)", 1000),
        Field("wheel_limit", "Teto manual de roda (m/s)", optional=True),
    )),
    ("Array QTR", (
        Field("sensor_count", "Quantidade de sensores", minimum=2, strictly_positive=False, integer=True),
        Field("sensor_board_width", "Largura da placa (mm)", 1000),
        Field("sensor_span", "Vão entre QTR extremos (mm)", 1000),
        Field("sensor_gap", "Extensão frontal do QTR (mm)", 1000, strictly_positive=False),
    )),
    ("Controle e IMU", (
        Field("acceleration_limit", "Rampa de aceleração (m/s²)"),
        Field("braking_limit", "Frenagem planejada (m/s²)"),
        Field("lateral_limit", "Limite lateral base (m/s²)"),
        Field("straight_kp_speed", "Redução do Kp na reta (m/s)", optional=True),
        Field("gyro_bias", "Viés do giroscópio (rad/s)", minimum=None),
    )),
    ("Mapeamento", (
        Field("mapping_speed", "Velocidade-alvo (m/s)"),
        Field("mapping_kp", "Kp", strictly_positive=False),
        Field("mapping_kd", "Kd", strictly_positive=False),
    )),
    ("Aderência", SUCTION_FIELDS),
)
ROBOT_FIELDS = {field.key: field for _, fields in ROBOT_GROUPS for field in fields}
SUCTION_KEYS = {field.key for field in SUCTION_FIELDS}
TURN_GAIN = Field("turn_feedforward", "Antecipação do giro (0–1)", strictly_positive=False, maximum=1)
CURVE_SPEED_PERCENT = Field("speed_percent", "Variação da velocidade (%)", minimum=-100, optional=True)
CURVE_NOTES = {side + level for side in ("D", "E") for level in CURVE_LEVELS if level != "Reta"}
PROFILE_FIELDS = (
    Field("v", "Velocidade nominal (m/s)"),
    Field("kp", "Kp", strictly_positive=False),
    Field("kd", "Kd", strictly_positive=False),
    TURN_GAIN,
)


def robot_from_form(values, *, suction_enabled=False):
    parsed = {}
    for key, field in ROBOT_FIELDS.items():
        try:
            parsed[key] = field.parse(values.get(key, ""))
        except ValueError:
            if suction_enabled or key not in SUCTION_KEYS:
                raise
            parsed[key] = None
    robot = Robot(**parsed, suction_enabled=suction_enabled)
    _ = robot.effective_lateral_limit
    if robot.sensor_span > robot.sensor_board_width:
        raise ValueError("A distância entre os QTR extremos deve caber na placa.")
    if robot.sensor_forward <= 0:
        raise ValueError("A fileira do QTR deve ficar à frente do eixo.")
    return robot


class ParameterEditor(ttk.Frame):
    def __init__(self, parent, safe, apply_robot, prepare_profile):
        super().__init__(parent, padding=10, style="Parameters.TFrame")
        self.safe = safe
        self.apply_robot = apply_robot
        self.prepare_profile = prepare_profile
        self.robot = None
        self.robot_editable = False
        self.base = None
        self.draft = {}
        self.curve_indices = []
        self.profile_editable = False
        self.robot_page = ttk.Frame(parent, padding=10, style="Parameters.TFrame")
        self.build_robot_form(self.robot_page)
        self.build_profile_form(self)

    def build_robot_form(self, page):
        actions = ttk.Frame(page, style="Parameters.TFrame")
        actions.pack(side="bottom", fill="x")
        self.robot_buttons = []
        for text, command in (("Aplicar ao simulador", self.submit_robot),
                              ("Abrir configuração", self.open_robot_form),
                              ("Salvar configuração", self.save_robot_form),
                              ("Preencher com padrões", lambda: self.fill_robot(Robot()))):
            button = ttk.Button(actions, text=text, command=lambda fn=command: self.safe(fn))
            button.pack(side="left", padx=(0, 5))
            self.robot_buttons.append(button)
        self.robot_summary = tk.StringVar()
        ttk.Label(page, textvariable=self.robot_summary, style="Parameters.TLabel").pack(
            side="bottom", anchor="w", pady=(0, 5))
        ttk.Label(page, text="Aplicar uma configuração diferente inicia uma nova sessão nesta pista. "
                  "Deixe o teto manual vazio para usar a rotação do motor e o Kp de reta vazio para desativar sua redução.",
                  wraplength=800, style="Parameters.TLabel").pack(anchor="w", pady=(0, 6))

        columns = ttk.Frame(page, style="Parameters.TFrame")
        columns.pack(fill="x")
        self.robot_vars = {}
        self.robot_entries = []
        self.suction_entries = []
        self.suction_enabled = tk.BooleanVar(value=False)
        layout = ((ROBOT_GROUPS[0], ROBOT_GROUPS[5]), (ROBOT_GROUPS[1], ROBOT_GROUPS[4]),
                  ROBOT_GROUPS[2:4])
        for column, groups in enumerate(layout):
            panel = ttk.Frame(columns, style="Parameters.TFrame")
            panel.grid(row=0, column=column, sticky="new", padx=(0, 8))
            columns.columnconfigure(column, weight=1, uniform="robot_params")
            for title, fields in groups:
                group = ttk.Labelframe(panel, text=title, padding=(8, 2), style="Parameters.TLabelframe")
                group.pack(fill="x", pady=(0, 4))
                group.columnconfigure(0, weight=1)
                suction_group = fields is SUCTION_FIELDS
                if suction_group:
                    self.suction_button = ttk.Checkbutton(
                        group, text="Sistema de sucção", variable=self.suction_enabled,
                        command=self.update_suction_form, style="Parameters.TCheckbutton")
                    self.suction_button.grid(row=0, column=0, columnspan=2, sticky="w", pady=(2, 3))
                for row, field in enumerate(fields, start=int(suction_group)):
                    var = self.robot_vars[field.key] = tk.StringVar()
                    ttk.Label(group, text=field.label, style="Parameters.TLabel").grid(row=row, column=0, sticky="w")
                    entry = ttk.Entry(group, textvariable=var, width=8, style="Parameters.TEntry")
                    entry.grid(row=row, column=1, sticky="e", padx=(4, 0), pady=1)
                    (self.suction_entries if suction_group else self.robot_entries).append(entry)

    def build_profile_form(self, page):
        self.profile_hint = tk.StringVar()
        heading = ttk.Frame(page, style="Parameters.TFrame")
        heading.pack(fill="x", pady=(0, 5))
        ttk.Label(heading, textvariable=self.profile_hint, style="Parameters.TLabel").pack(side="left")
        self.discard_button = ttk.Button(heading, text="Descartar rascunho", command=self.discard_draft)
        self.discard_button.pack(side="right")
        editor = ttk.Frame(page, style="Parameters.TFrame")
        editor.pack(side="bottom", fill="x", pady=(7, 0))
        self.profile_vars = {}
        self.profile_entries = []
        for field in PROFILE_FIELDS:
            var = self.profile_vars[field.key] = tk.StringVar()
            ttk.Label(editor, text=field.label, style="Parameters.TLabel").pack(side="left", padx=(0, 4))
            entry = ttk.Entry(editor, width=8, textvariable=var, style="Parameters.TEntry")
            entry.pack(side="left", padx=(0, 8))
            self.profile_entries.append(entry)
        self.save_sector_button = ttk.Button(editor, text="Guardar trecho", command=lambda: self.safe(self.save_sector))
        self.save_sector_button.pack(side="left")
        self.prepare_button = ttk.Button(editor, text="Preparar validação",
                                         command=lambda: self.safe(self.submit_profile))
        self.prepare_button.pack(side="right")
        group = ttk.Labelframe(page, text="Todas as curvas (D1–D5 e E1–E5)", padding=(7, 3),
                               style="Parameters.TLabelframe")
        group.pack(side="bottom", fill="x", pady=(5, 0))
        self.curve_gain = tk.StringVar()
        self.curve_percent = tk.StringVar()
        self.curve_entries = []
        for label, var in (("Antecipação (0–1)", self.curve_gain),
                           ("Velocidade (% sobre o perfil em uso)", self.curve_percent)):
            ttk.Label(group, text=label, style="Parameters.TLabel").pack(side="left", padx=(0, 4))
            entry = ttk.Entry(group, textvariable=var, width=8, style="Parameters.TEntry")
            entry.pack(side="left", padx=(0, 10))
            self.curve_entries.append(entry)
        self.save_curves_button = ttk.Button(group, text="Guardar nas curvas",
                                             command=lambda: self.safe(self.save_curves))
        self.save_curves_button.pack(side="left")
        table_frame = ttk.Frame(page, style="Parameters.TFrame")
        table_frame.pack(fill="both", expand=True)
        columns = ("Trecho", "Nota", "Nominal (m/s)", "Kp", "Kd", "Antecipação", "Ajuste")
        self.profile_table = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="browse")
        for label in columns:
            self.profile_table.heading(label, text=label)
            self.profile_table.column(label, width=87, anchor="center")
        scrollbar = ttk.Scrollbar(table_frame, command=self.profile_table.yview)
        scrollbar.pack(side="right", fill="y")
        self.profile_table.configure(yscrollcommand=scrollbar.set)
        self.profile_table.pack(fill="both", expand=True)
        self.profile_table.bind("<<TreeviewSelect>>", self.load_sector)
        for var in (self.curve_gain, self.curve_percent):
            var.trace_add("write", lambda *_: self.update_profile_form())

    def sync(self, robot, profile, *, robot_editable, profile_editable):
        self.robot_editable = robot_editable
        if robot is not self.robot:
            self.robot = robot
            self.fill_robot(robot)
            self.robot_summary.set(
                f"Em uso · gabarito {robot.overall_length*1000:.0f} × {robot.overall_width*1000:.0f} × "
                f"{robot.body_height*1000:.0f} mm · QTR a {robot.sensor_forward*1000:.0f} mm do eixo · "
                f"teto de {robot.max_wheel_speed:.2f} m/s por roda · "
                f"limite lateral de {robot.effective_lateral_limit:.2f} m/s²".replace(".", ",")
            )
        if profile is not self.base:
            self.base = profile
            self.draft.clear()
            self.curve_indices = [i for i, sector in enumerate(profile["segments"])
                                  if sector.get("label") in CURVE_NOTES] if profile else []
            self.curve_gain.set("")
            self.curve_percent.set("")
            self.profile_table.delete(*self.profile_table.get_children())
            for i in range(len(profile["segments"]) if profile else 0):
                self.profile_table.insert("", "end", iid=str(i), values=self.sector_row(i))
            if profile:
                self.profile_table.selection_set("0")
            self.load_sector()
        self.profile_editable = profile_editable
        for widget in (*self.robot_entries, *self.robot_buttons):
            widget.state(("!disabled",) if robot_editable else ("disabled",))
        self.update_suction_form()
        self.update_profile_form()

    def update_profile_form(self):
        editable = self.profile_editable and bool(self.profile_table.selection())
        for widget in (*self.profile_entries, self.save_sector_button, self.prepare_button, self.discard_button):
            widget.state(("!disabled",) if editable else ("disabled",))
        for entry in self.curve_entries:
            entry.state(("!disabled",) if editable and self.curve_indices else ("disabled",))
        has_batch = self.curve_gain.get().strip() or self.curve_percent.get().strip()
        self.save_curves_button.state(("!disabled",) if editable and self.curve_indices and has_batch else ("disabled",))
        if self.base is None:
            self.profile_hint.set("Gere um mapa para editar os parâmetros por trecho.")
        elif not self.profile_editable:
            self.profile_hint.set("A edição estará disponível depois do teste ou da validação pendente.")
        else:
            self.profile_hint.set(f"{len(self.draft)} trecho(s) editado(s). Guarde cada trecho antes de selecionar outro.")

    def fill_robot(self, robot):
        for key, field in ROBOT_FIELDS.items():
            self.robot_vars[key].set(field.display(getattr(robot, key)))
        self.suction_enabled.set(robot.suction_enabled)
        self.update_suction_form()

    def update_suction_form(self):
        self.suction_button.state(("!disabled",) if self.robot_editable else ("disabled",))
        enabled = self.robot_editable and self.suction_enabled.get()
        for entry in self.suction_entries:
            entry.state(("!disabled",) if enabled else ("disabled",))

    def robot_form(self):
        return robot_from_form({key: var.get() for key, var in self.robot_vars.items()},
                               suction_enabled=self.suction_enabled.get())

    def submit_robot(self):
        self.apply_robot(self.robot_form())

    def open_robot_form(self):
        path = filedialog.askopenfilename(filetypes=[("Configuração JSON", "*.json")])
        if not path:
            return
        data = read_json(path)
        if not isinstance(data, dict):
            raise ValueError("O arquivo deve conter uma configuração de robô em JSON.")
        defaults = asdict(Robot())
        if not ROBOT_FIELDS.keys() & data.keys() and "suction_enabled" not in data:
            raise ValueError("O arquivo não contém parâmetros reconhecidos do robô.")
        enabled = data.get("suction_enabled", False)
        if type(enabled) is not bool:
            raise ValueError("A opção de sucção deve ser true ou false no JSON.")
        values = {}
        for key, field in ROBOT_FIELDS.items():
            value = data.get(key, defaults[key])
            if value is not None and type(value) not in (int, float):
                raise ValueError(f"{field.label} deve ser um número no JSON.")
            values[key] = field.display(value)
        self.fill_robot(robot_from_form(values, suction_enabled=enabled))

    def save_robot_form(self):
        robot = self.robot_form()
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="robo.json",
                                            filetypes=[("Configuração JSON", "*.json")])
        if path:
            write_json(path, asdict(robot))

    def sector_values(self, index):
        # Perfis anteriores não trazem o ganho de antecipação.
        return {"turn_feedforward": 0.0, **self.base["segments"][index], **self.draft.get(index, {})}

    def sector_row(self, index):
        values = self.sector_values(index)
        return (index+1, values.get("label", "—"), *(field.display(values[field.key]) for field in PROFILE_FIELDS),
                "Editado" if index in self.draft else "")

    def load_sector(self, _=None):
        selection = self.profile_table.selection()
        index = int(selection[0]) if selection and self.base else None
        values = self.sector_values(index) if index is not None else None
        for field in PROFILE_FIELDS:
            self.profile_vars[field.key].set(field.display(values[field.key]) if values else "")
        self.update_profile_form()

    def save_sector(self):
        selection = self.profile_table.selection()
        if not selection or self.base is None:
            raise ValueError("Selecione um trecho para editar.")
        index = int(selection[0])
        previous = self.sector_values(index)
        raw = {field.key: self.profile_vars[field.key].get() for field in PROFILE_FIELDS}
        values = {field.key: (previous[field.key] if raw[field.key] == field.display(previous[field.key])
                              else field.parse(raw[field.key])) for field in PROFILE_FIELDS}
        self.store_sector(index, values)
        self.update_profile_form()

    def store_sector(self, index, values):
        sector = self.base["segments"][index]
        changes = self.draft.get(index, {}).copy()
        for key, value in values.items():
            if value == sector.get(key, 0.0):
                changes.pop(key, None)
            else:
                changes[key] = value
        if changes:
            self.draft[index] = changes
        else:
            self.draft.pop(index, None)
        self.profile_table.item(str(index), values=self.sector_row(index))

    def save_curves(self):
        if not self.profile_editable or not self.curve_indices:
            raise ValueError("Selecione um perfil editável com curvas mapeadas.")
        raw_gain = self.curve_gain.get().strip()
        gain = TURN_GAIN.parse(raw_gain) if raw_gain else None
        percent = CURVE_SPEED_PERCENT.parse(self.curve_percent.get())
        if gain is None and percent is None:
            raise ValueError("Informe a antecipação ou a variação de velocidade para as curvas.")
        adjustments = {}
        for index in self.curve_indices:
            values = {}
            if gain is not None:
                values[TURN_GAIN.key] = gain
            if percent is not None:
                speed = self.base["segments"][index]["v"] * (1 + percent / 100)
                if not isfinite(speed) or speed <= 0:
                    raise ValueError(f"O ajuste gera velocidade inválida no trecho {index+1}.")
                values["v"] = speed
            adjustments[index] = values
        self.save_sector()
        for index, values in adjustments.items():
            self.store_sector(index, values)
        self.curve_gain.set("")
        self.curve_percent.set("")
        self.load_sector()

    def discard_draft(self):
        changed = list(self.draft)
        self.draft.clear()
        for index in changed:
            self.profile_table.item(str(index), values=self.sector_row(index))
        self.curve_gain.set("")
        self.curve_percent.set("")
        self.load_sector()

    def has_unsaved_profile(self):
        if self.draft or self.curve_gain.get().strip() or self.curve_percent.get().strip():
            return True
        selected = self.profile_table.selection()
        if self.base is None or not selected:
            return False
        values = self.sector_values(int(selected[0]))
        return any(self.profile_vars[field.key].get() != field.display(values[field.key]) for field in PROFILE_FIELDS)

    def submit_profile(self):
        if self.curve_gain.get().strip() or self.curve_percent.get().strip():
            raise ValueError("Clique em Guardar nas curvas ou limpe os campos do lote antes de preparar a validação.")
        self.save_sector()
        if not self.draft:
            raise ValueError("Altere pelo menos um parâmetro do perfil antes de preparar a validação.")
        candidate = deepcopy(self.base)
        for index, values in self.draft.items():
            candidate["segments"][index].update(values)
        self.prepare_profile(candidate)
