"""Dashboard UI for mission_fsm.

Launched automatically in a background thread when the FSM node starts.
The dashboard communicates with FSMNode through direct attribute/method
calls — no ROS topics needed for the UI itself.
"""

import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox


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

# Boundary-only interior blocks R1 KFS may not sit on (rule 3.3.3), and the
# entrance row the Fake KFS may not sit on (rule 4.1.4) — same rules
# path.py / forest_planner.py enforce.
_INTERIOR_BLOCKS = {5, 8}
_ENTRANCE_ROW = {1, 2, 3}


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _parse_blocks(text: str, count: int) -> list[int]:
    """Parse a comma/space separated string of block numbers.

    Raises ValueError with a human-readable message on any problem.
    """
    raw = text.replace(",", " ").split()
    if len(raw) != count:
        raise ValueError(f"Expected {count} block number(s), got {len(raw)}.")
    try:
        blocks = [int(x) for x in raw]
    except ValueError:
        raise ValueError("Block numbers must be integers.")
    if len(set(blocks)) != count:
        raise ValueError("Block numbers must be distinct.")
    if any(b < 1 or b > 12 for b in blocks):
        raise ValueError("Block numbers must be between 1 and 12.")
    return blocks


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

        tk.Label(rp, text="FOREST STATE — AREA 2",
                 font=self._large, bg=BG_DARK, fg=TEXT_WHITE).pack(pady=(0, 8))

        self._build_forest_card(rp)

    def _build_forest_card(self, parent):
        """Card for typing in the Forest state (R1/R2 KFS + Fake block) and
        pushing it to FSMNode.set_forest_state(), which Area2State picks up
        once nav arrives at the Forest entrance."""
        card = tk.Frame(parent, bg=BG_PANEL, padx=16, pady=14,
                        highlightbackground=GREEN, highlightthickness=2)
        card.pack(fill=tk.X, pady=4)

        self._forest_entries: dict[str, tk.StringVar] = {}

        field_specs = [
            ("r1", "R1 KFS (3 blocks, boundary only — e.g. 1 3 10)", "1 3 10"),
            ("r2", "R2 KFS (4 blocks — e.g. 5 6 8 11)",              "5 6 8 11"),
            ("fake", "Fake KFS (1 block, not 1/2/3 — e.g. 9)",       "9"),
        ]

        for key, label_text, placeholder in field_specs:
            row = tk.Frame(card, bg=BG_PANEL)
            row.pack(fill=tk.X, pady=4)

            tk.Label(row, text=label_text, font=("Arial", 9),
                     bg=BG_PANEL, fg=TEXT_DIM, width=38, anchor=tk.W
                     ).pack(side=tk.LEFT)

            var = tk.StringVar(value="")
            ent = tk.Entry(row, textvariable=var, width=16,
                           bg=BG_CARD, fg=TEXT_WHITE,
                           insertbackground=TEXT_WHITE,
                           relief=tk.FLAT, font=self._mono)
            ent.pack(side=tk.LEFT, padx=8)
            self._forest_entries[key] = var

        btn_row = tk.Frame(card, bg=BG_PANEL)
        btn_row.pack(fill=tk.X, pady=(10, 0))

        self.lbl_forest_msg = tk.Label(btn_row, text="", font=("Arial", 9),
                                       bg=BG_PANEL, fg=GREEN)
        self.lbl_forest_msg.pack(side=tk.LEFT)

        tk.Button(btn_row, text="SET FOREST STATE",
                  bg=GREEN, fg=BG_DARK, font=self._bold,
                  width=20, height=1, relief=tk.FLAT,
                  cursor="hand2", command=self._on_set_forest
                  ).pack(side=tk.RIGHT)

        # ── Live readout of whatever FSMNode currently has stored ────────────
        readout_frame = tk.Frame(parent, bg=BG_DARK)
        readout_frame.pack(fill=tk.X, pady=(8, 0))

        tk.Label(readout_frame, text="CURRENTLY SET:", font=("Arial", 8),
                 bg=BG_DARK, fg=TEXT_DIM).pack(anchor=tk.W)
        self.lbl_forest_current = tk.Label(readout_frame, text="(nothing set yet)",
                                           font=self._mono, bg=BG_DARK, fg=TEXT_WHITE,
                                           justify=tk.LEFT, anchor=tk.W)
        self.lbl_forest_current.pack(anchor=tk.W)

    # ── Button callbacks ──────────────────────────────────────────────────────

    def _on_start(self):
        self.fsm.trigger_start()

    def _on_retry(self, area_id: int):
        self.fsm.trigger_retry_area(area_id)

    def _on_stop(self):
        self.fsm.trigger_stop()

    def _on_reset(self):
        self.fsm.trigger_reset()

    def _on_set_forest(self):
        """Validate the typed-in Forest state and push it to FSMNode."""
        try:
            r1 = _parse_blocks(self._forest_entries["r1"].get(), 3)
            r2 = _parse_blocks(self._forest_entries["r2"].get(), 4)
            fake = _parse_blocks(self._forest_entries["fake"].get(), 1)[0]
        except ValueError as e:
            messagebox.showerror("Invalid Forest state", str(e))
            return

        bad_interior = sorted(set(r1) & _INTERIOR_BLOCKS)
        if bad_interior:
            messagebox.showerror(
                "Invalid Forest state",
                f"R1 KFS must be on boundary blocks (rule 3.3.3); "
                f"{bad_interior} are interior and not allowed.",
            )
            return

        if fake in _ENTRANCE_ROW:
            messagebox.showerror(
                "Invalid Forest state",
                "Fake KFS cannot be on entrance blocks 1, 2, or 3 (rule 4.1.4).",
            )
            return

        all_blocks = r1 + r2 + [fake]
        if len(set(all_blocks)) != len(all_blocks):
            messagebox.showerror(
                "Invalid Forest state",
                "R1, R2, and Fake blocks must all be distinct from each other.",
            )
            return

        self.fsm.set_forest_state(r1, r2, fake)

        self.lbl_forest_msg.config(
            text=f"Set! R1={r1}  R2={r2}  Fake={fake}",
            fg=GREEN,
        )
        self.window.after(4000, lambda: self.lbl_forest_msg.config(text=""))

    # ── Periodic refresh ──────────────────────────────────────────────────────

    def _refresh(self):
        if self._stop.is_set():
            return

        # Update state badge
        state = getattr(self.fsm.task, "current_state", "—")
        color = STATE_COLORS.get(state, TEXT_WHITE)
        self.lbl_state.config(text=state, fg=color)

        # Update Forest-state readout from whatever's actually stored on FSMNode
        r1 = getattr(self.fsm, "r1_blocks", [])
        r2 = getattr(self.fsm, "r2_blocks", [])
        fake = getattr(self.fsm, "fake_block", 0)
        if r1 or r2 or fake:
            self.lbl_forest_current.config(text=f"R1={r1}  R2={r2}  Fake={fake}")
        else:
            self.lbl_forest_current.config(text="(nothing set yet)")

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