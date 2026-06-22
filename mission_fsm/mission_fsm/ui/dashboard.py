"""Dashboard UI for mission_fsm.

Launched automatically in a background thread when the FSM node starts.
The dashboard communicates with FSMNode through direct attribute/method
calls — no ROS topics needed for the UI itself.
"""

import math
import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox

import yaml

from ..config.loader import _CONFIG_PATH


# ──────────────────────────────────────────────────────────────────────────────
# Colour palette
# ──────────────────────────────────────────────────────────────────────────────
BG_DARK    = "#1a1a2e"
BG_PANEL   = "#16213e"
BG_CARD    = "#0f3460"
ACCENT_1   = "#e94560"   # red-ish accent
ACCENT_2   = "#533483"   # purple accent
GREEN      = "#00c896"
BLUE       = "#4285f4"
ORANGE     = "#ff9900"
GREY       = "#3a3a5c"
TEXT_WHITE = "#f0f0f0"
TEXT_DIM   = "#8888aa"

STATE_COLORS = {
    "IDLE":   "#8888aa",
    "AREA_1": "#4285f4",
    "AREA_2": "#00c896",
    "AREA_3": "#ff9900",
    "DONE":   "#e94560",
}


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _save_yaml(data: dict) -> None:
    """Write goal data back to areas.yaml (keeps comments stripped by yaml.dump)."""
    header = (
        "# ============================================================\n"
        "#  Area Goal Coordinates  —  edit then press APPLY\n"
        "#\n"
        "#  x   : metres along the map X axis\n"
        "#  y   : metres along the map Y axis\n"
        "#  yaw : final heading in RADIANS (0=East, 1.5708=North)\n"
        "# ============================================================\n\n"
    )
    with open(_CONFIG_PATH, "w") as f:
        f.write(header)
        yaml.dump(data, f, default_flow_style=False, sort_keys=True)


def _load_yaml() -> dict:
    with open(_CONFIG_PATH, "r") as f:
        return yaml.safe_load(f) or {}


# ──────────────────────────────────────────────────────────────────────────────
# Dashboard class
# ──────────────────────────────────────────────────────────────────────────────

class RobotDashboard:
    """Main dashboard window for KRAI 2026 FSM control."""

    def __init__(self, window: tk.Tk, fsm_node):
        print("DEBUG: RobotDashboard __init__ start", flush=True)
        self.window  = window
        self.fsm     = fsm_node
        self._stop   = threading.Event()

        window.title("KRAI 2026 — MASTER CONTROL DASHBOARD")
        window.geometry("900x520")
        window.resizable(False, False)
        window.configure(bg=BG_DARK)

        # Load fonts
        print("DEBUG: Loading fonts...", flush=True)
        self._bold  = tkfont.Font(family="Arial", size=10, weight="bold")
        self._large = tkfont.Font(family="Arial", size=13, weight="bold")
        self._mono  = tkfont.Font(family="Courier", size=10)

        print("DEBUG: Building header...", flush=True)
        self._build_header()
        print("DEBUG: Building left panel...", flush=True)
        self._build_left_panel()
        print("DEBUG: Building right panel...", flush=True)
        self._build_right_panel()

        print("DEBUG: Starting refresh...", flush=True)
        self._refresh()
        print("DEBUG: RobotDashboard __init__ end", flush=True)

    # ── Layout builders ───────────────────────────────────────────────────────

    def _build_header(self):
        hdr = tk.Frame(self.window, bg=BG_CARD, height=50)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="KRAI 2026 - MASTER CONTROL DASHBOARD",
                 font=("Arial", 14, "bold"), bg=BG_CARD, fg=TEXT_WHITE
                 ).pack(side=tk.LEFT, padx=20, pady=12)

        # Live clock
        self.lbl_clock = tk.Label(hdr, text="", font=self._mono,
                                  bg=BG_CARD, fg=TEXT_DIM)
        self.lbl_clock.pack(side=tk.RIGHT, padx=20)

    def _build_left_panel(self):
        lp = tk.Frame(self.window, bg=BG_PANEL, width=240)
        lp.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 2))
        lp.pack_propagate(False)

        tk.Label(lp, text="COMMAND CONTROL", font=self._large,
                 bg=BG_PANEL, fg=TEXT_WHITE).pack(pady=(20, 10))

        def btn(parent, text, color, cmd, h=2):
            b = tk.Button(parent, text=text, bg=color, fg="white",
                          font=self._bold, width=24, height=h,
                          activebackground=color, relief=tk.FLAT,
                          cursor="hand2", command=cmd)
            b.pack(pady=4, padx=12)
            return b

        btn(lp, "START AUTO",  GREEN,   self._on_start, h=2)

        sep = tk.Frame(lp, bg=GREY, height=1)
        sep.pack(fill=tk.X, padx=12, pady=6)

        btn(lp, "RETRY AREA 1", BLUE,    lambda: self._on_retry(1), h=1)
        btn(lp, "RETRY AREA 2", BLUE,    lambda: self._on_retry(2), h=1)
        btn(lp, "RETRY AREA 3", BLUE,    lambda: self._on_retry(3), h=1)

        sep2 = tk.Frame(lp, bg=GREY, height=1)
        sep2.pack(fill=tk.X, padx=12, pady=6)

        btn(lp, "EMERGENCY STOP", ACCENT_1, self._on_stop,  h=2)
        btn(lp, "RESET SYSTEM",   ACCENT_2, self._on_reset, h=1)

        # ── State badge ───────────────────────────────────────────────────────
        badge_frame = tk.Frame(lp, bg=BG_PANEL)
        badge_frame.pack(side=tk.BOTTOM, pady=20)

        tk.Label(badge_frame, text="CURRENT STATE", font=("Arial", 8),
                 bg=BG_PANEL, fg=TEXT_DIM).pack()
        self.lbl_state = tk.Label(badge_frame, text="IDLE",
                                  font=("Arial", 16, "bold"),
                                  bg=BG_PANEL, fg=GREEN)
        self.lbl_state.pack()


    def _build_right_panel(self):
        rp = tk.Frame(self.window, bg=BG_DARK)
        rp.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        tk.Label(rp, text="AREA GOAL COORDINATES",
                 font=self._large, bg=BG_DARK, fg=TEXT_WHITE).pack(pady=(0, 8))

        # ── Coordinate editor cards ───────────────────────────────────────────
        self._entries: dict[str, dict[str, tk.Entry]] = {}

        areas_cfg = _load_yaml()
        area_colors = {"area_1": BLUE, "area_2": GREEN, "area_3": ORANGE}

        for area_key, color in area_colors.items():
            area_data = areas_cfg.get(area_key, {"x": 0.0, "y": 0.0, "yaw": 0.0})
            self._build_area_card(rp, area_key, area_data, color)

        # ── Apply button ──────────────────────────────────────────────────────
        apply_row = tk.Frame(rp, bg=BG_DARK)
        apply_row.pack(fill=tk.X, pady=(12, 0))

        self.lbl_apply_msg = tk.Label(apply_row, text="", font=("Arial", 9),
                                      bg=BG_DARK, fg=GREEN)
        self.lbl_apply_msg.pack(side=tk.LEFT, padx=10)

        tk.Button(apply_row, text="APPLY & SAVE COORDINATES",
                  bg=GREEN, fg=BG_DARK, font=self._bold,
                  width=28, height=1, relief=tk.FLAT,
                  cursor="hand2", command=self._on_apply
                  ).pack(side=tk.RIGHT, padx=10)

    def _build_area_card(self, parent, area_key: str, data: dict, color: str):
        """Build one coordinate-editor card for an area."""
        label = area_key.replace("_", " ").upper()

        card = tk.Frame(parent, bg=BG_PANEL, padx=12, pady=8,
                        highlightbackground=color, highlightthickness=2)
        card.pack(fill=tk.X, pady=4)

        # Title strip
        title_row = tk.Frame(card, bg=BG_PANEL)
        title_row.pack(fill=tk.X)
        tk.Label(title_row, text=f"[ {label} ]", font=self._bold,
                 bg=BG_PANEL, fg=color).pack(side=tk.LEFT)


        # Fields: X  Y  YAW(deg display, stored as rad)
        fields_row = tk.Frame(card, bg=BG_PANEL)
        fields_row.pack(fill=tk.X, pady=(6, 0))

        entries = {}
        field_specs = [
            ("X  (m)", "x"),
            ("Y  (m)", "y"),
            ("Yaw (°)", "yaw_deg"),
        ]

        for lbl_text, key in field_specs:
            col = tk.Frame(fields_row, bg=BG_PANEL)
            col.pack(side=tk.LEFT, padx=8)

            tk.Label(col, text=lbl_text, font=("Arial", 8),
                     bg=BG_PANEL, fg=TEXT_DIM).pack(anchor=tk.W)

            # Yaw: show in degrees for human readability
            if key == "yaw_deg":
                init_val = round(math.degrees(float(data.get("yaw", 0.0))), 2)
            else:
                init_val = float(data.get(key, 0.0))

            var = tk.StringVar(value=str(init_val))
            ent = tk.Entry(col, textvariable=var, width=9,
                           bg=BG_CARD, fg=TEXT_WHITE,
                           insertbackground=TEXT_WHITE,
                           relief=tk.FLAT, font=self._mono)
            ent.pack()
            entries[key] = var

        self._entries[area_key] = entries

    # ── Button callbacks ──────────────────────────────────────────────────────

    def _on_start(self):
        self.fsm.trigger_start()

    def _on_retry(self, area_id: int):
        self.fsm.trigger_retry_area(area_id)

    def _on_stop(self):
        self.fsm.trigger_stop()

    def _on_reset(self):
        self.fsm.trigger_reset()

    def _on_apply(self):
        """Read entry fields, validate, convert yaw° → rad, write YAML."""
        new_cfg = {}
        try:
            for area_key, fields in self._entries.items():
                x   = float(fields["x"].get())
                y   = float(fields["y"].get())
                yaw_rad = math.radians(float(fields["yaw_deg"].get()))
                new_cfg[area_key] = {
                    "x":   round(x, 6),
                    "y":   round(y, 6),
                    "yaw": round(yaw_rad, 6),
                }
        except ValueError as e:
            messagebox.showerror("Invalid input", f"Please enter valid numbers.\n\n{e}")
            return

        _save_yaml(new_cfg)
        # Push new values into the live loader dict so running states see them
        from ..config import loader as _loader
        _loader.AREA_GOALS.update(new_cfg)
        # Also refresh each area state's _GOAL cache
        self._reload_state_goals()

        self.lbl_apply_msg.config(
            text="Saved! Restart areas to apply.",
            fg=GREEN,
        )
        self.window.after(4000, lambda: self.lbl_apply_msg.config(text=""))

    def _reload_state_goals(self):
        """Tell each area state module to reload its _GOAL from AREA_GOALS."""
        try:
            from ..task.states import area_1, area_2, area_3
            from ..config.loader import AREA_GOALS
            area_1._GOAL = AREA_GOALS.get("area_1", area_1._GOAL)
            area_2._GOAL = AREA_GOALS.get("area_2", area_2._GOAL)
            area_3._GOAL = AREA_GOALS.get("area_3", area_3._GOAL)
        except Exception:
            pass  # non-fatal; node restart will always pick up changes

    # ── Periodic refresh ──────────────────────────────────────────────────────

    def _refresh(self):
        if self._stop.is_set():
            return

        # Update state badge
        state = getattr(self.fsm.task, "current_state", "—")
        color = STATE_COLORS.get(state, TEXT_WHITE)
        self.lbl_state.config(text=state, fg=color)

        # Update clock
        import datetime
        now = datetime.datetime.now().strftime("%H:%M:%S")
        self.lbl_clock.config(text=now)

        self.window.after(200, self._refresh)

    def stop(self):
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# Launch helpers
# ──────────────────────────────────────────────────────────────────────────────

def run_dashboard_main_thread(fsm_node) -> None:
    """Run the Tkinter dashboard on the main thread, spinning ROS 2 synchronously.

    This keeps the GUI and ROS 2 execution in a single thread, avoiding all
    background thread race conditions, C-extension conflicts, and X11/Tkinter
    threading/signal crashes (stack smashing).
    """
    import rclpy

    root = tk.Tk()
    app = RobotDashboard(root, fsm_node)

    # Periodic step to spin ROS 2 callbacks
    def ros_spin_step():
        if rclpy.ok():
            try:
                rclpy.spin_once(fsm_node, timeout_sec=0.0)
            except Exception as e:
                fsm_node.get_logger().error(f"Error in ROS spin: {e}")
        root.after(10, ros_spin_step)

    # Start the periodic spin step
    root.after(10, ros_spin_step)

    root.protocol("WM_DELETE_WINDOW", root.quit)
    try:
        root.mainloop()
    finally:
        app.stop()


def launch_dashboard(fsm_node) -> threading.Thread:
    """Spawn the Tkinter dashboard in a daemon thread (legacy helper).

    .. deprecated::
        Prefer :func:`run_dashboard_main_thread` which correctly keeps
        Tkinter on the main thread.  This function is kept only for
        environments where the main thread is not available.
    """
    def _run():
        root = tk.Tk()
        app = RobotDashboard(root, fsm_node)
        root.protocol("WM_DELETE_WINDOW", root.quit)
        root.mainloop()
        app.stop()

    t = threading.Thread(target=_run, name="dashboard-ui", daemon=True)
    t.start()
    return t

