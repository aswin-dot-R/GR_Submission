#!/usr/bin/env python3
"""GR Assignment — Demo Launcher (click-to-run GUI).

Runs on the host; drives the Jazzy / Gazebo-Harmonic container via docker compose.
Buttons launch each section and open its output. Sections 1&2 (MoveIt) and
Section 3 (Gazebo) can't share the container at once, so the Start buttons clean
up the other first.

Run:  python3 scripts/demo_gui.py
"""
import os
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMPOSE = ["docker", "compose", "-f", os.path.join(REPO, "docker", "docker-compose.yml")]
WS = "source /opt/ros/jazzy/setup.bash; source /home/dev/ros2_ws/install_jazzy/setup.bash"
INST = "/home/dev/ros2_ws/install_jazzy"
DATA = os.path.join(REPO, "data")

KILL = ("pkill -9 -f 'gz sim|move_group|ros2 launch|rviz2|robot_state_publisher|"
        "wiping_controller|ruby|ros2_control_node|reachability_sweep|"
        "coverage_planner|parameter_bridge|traj_player' 2>/dev/null; sleep 2")


def jazzy(cmd, detached=False):
    return COMPOSE + ["exec", "-T"] + (["-d"] if detached else []) + ["jazzy", "bash", "-lc", cmd]


class App:
    def __init__(self, root):
        self.root = root
        root.title("GR Assignment — Demo Launcher")
        root.geometry("760x560")

        head = tk.Label(root, text="AgileX Piper — 3-Section Demo (Jazzy / Gazebo Harmonic)",
                        font=("Sans", 13, "bold"))
        head.pack(pady=8)

        body = tk.Frame(root); body.pack(fill="x", padx=10)

        # --- Sections 1, 2 & 3 (MoveIt) ---
        f12 = tk.LabelFrame(body, text=" Sections 1, 2 & 3 — MoveIt ",
                            font=("Sans", 10, "bold"), padx=8, pady=8)
        f12.pack(fill="x", pady=6)
        r1 = tk.Frame(f12); r1.pack(fill="x")
        self._btn(r1, "① Start MoveIt", self.start_moveit, "#2d6cdf")
        self._btn(r1, "Run Reachability sweep", self.run_reach)
        self._btn(r1, "Show heatmap", lambda: self.show("reachability.png"), "#444")
        self._btn(r1, "Run Coverage", self.run_cov)
        self._btn(r1, "Show coverage", lambda: self.show("coverage_path.png"), "#444")
        r2 = tk.Frame(f12); r2.pack(fill="x", pady=(6, 0))
        tk.Label(r2, text="Sec 3 (force):", font=("Sans", 9)).pack(side="left", padx=(2, 4))
        self._btn(r2, "③ Wipe — counter (10 N)", self.run_wipe_mi, "#2d6cdf")
        self._btn(r2, "③ Wipe — mirror (6 N)", self.run_wipe_mi_mirror, "#2d6cdf")
        self._btn(r2, "Show force (counter)", lambda: self.show("wiping_log.png"), "#444")
        self._btn(r2, "Show force (mirror)", lambda: self.show("wiping_log_mirror.png"), "#444")

        # --- Section 3 (Gazebo physics — secondary demo) ---
        f3 = tk.LabelFrame(body, text=" Section 3 — Gazebo physics (secondary demo) ",
                           font=("Sans", 10, "bold"), padx=8, pady=8)
        f3.pack(fill="x", pady=6)
        self._btn(f3, "③ Start Gazebo (GUI)", self.start_gz, "#1f9d55")
        self._btn(f3, "Wipe gz (counter)", self.run_wipe)
        self._btn(f3, "Wipe gz (mirror)", self.run_wipe_mirror)
        self._btn(f3, "Replay full path", self.run_gz_continuous, "#1f9d55")
        self._btn(f3, "Loop full path", self.loop_gz_continuous, "#1f9d55")
        self._btn(f3, "Show counter plot", lambda: self.show("wiping_gz_counter.png"), "#444")
        self._btn(f3, "Show mirror plot", lambda: self.show("wiping_gz_mirror.png"), "#444")

        # --- global ---
        fg = tk.Frame(body); fg.pack(fill="x", pady=6)
        self._btn(fg, "▶ Run ALL (headless)", self.run_all, "#7048e8")
        self._btn(fg, "Open outputs folder", self.open_outputs, "#444")
        self._btn(fg, "■ Stop everything", self.stop, "#d64545")

        tk.Label(root, text="Status log:", anchor="w").pack(fill="x", padx=10)
        self.log = scrolledtext.ScrolledText(root, height=14, bg="#10141a", fg="#d7e0ea",
                                             font=("Mono", 9))
        self.log.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.say("Ready. Tip: ① Start MoveIt for Sec 1&2, or ③ Start Gazebo for Sec 3.")

    # ---------- helpers ----------
    def _btn(self, parent, text, cmd, color="#3367d6"):
        b = tk.Button(parent, text=text, command=cmd, bg=color, fg="white",
                      activebackground=color, relief="flat", padx=10, pady=6, font=("Sans", 10))
        b.pack(side="left", padx=4)
        return b

    def say(self, msg):
        self.root.after(0, lambda: (self.log.insert("end", msg + "\n"), self.log.see("end")))

    def run(self, label, cmd, detached=False):
        self.say(f"▶ {label} …")

        def worker():
            try:
                if detached:
                    subprocess.run(jazzy(cmd, True), cwd=REPO, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=60)
                    self.say(f"  {label}: launched (running in background)")
                else:
                    p = subprocess.Popen(jazzy(cmd), cwd=REPO, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True)
                    for line in p.stdout:
                        s = line.rstrip()
                        # show live progress + results, skip pure ROS boilerplate
                        keys = ("cells (", "%)", "Reachable", "Wrote", "Surface-aligned",
                                "coverage=", "geom_coverage", "->", "wrote", "complete",
                                "up (", "Loaded", "best_yaw", "ERROR", "Error", "error")
                        if any(k in s for k in keys):
                            # strip the ros log prefix [...] [..] [node]: for readability
                            i = s.rfind("]: ")
                            self.say("    " + (s[i + 3:] if i != -1 else s))
                    p.wait()
                    self.say(f"  {label}: done")
            except Exception as e:
                self.say(f"  {label}: ERROR {e}")

        threading.Thread(target=worker, daemon=True).start()

    # ---------- run a host .sh script, streaming its output to the log ----------
    def script(self, label, path):
        self.say(f"▶ {label} …")

        def worker():
            try:
                p = subprocess.Popen(["bash", os.path.join(REPO, path)], cwd=REPO,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for line in p.stdout:
                    self.say("    " + line.rstrip())
                p.wait()
                self.say(f"  {label}: {'done' if p.returncode == 0 else 'FAILED (see log)'}")
            except Exception as e:
                self.say(f"  {label}: ERROR {e}")

        threading.Thread(target=worker, daemon=True).start()

    # ---------- actions (each button runs its scripts/gui/*.sh) ----------
    def start_moveit(self):
        self.script("Start MoveIt + RViz + scene (RViz window in ~25-30 s)", "scripts/gui/start_moveit.sh")

    def run_reach(self):
        self.script("Section 1 reachability sweep (~1-2 min)", "scripts/gui/reachability.sh")

    def run_cov(self):
        self.script("Section 2 coverage planner", "scripts/gui/coverage.sh")

    def start_gz(self):
        self.script("Start Gazebo Harmonic (GUI ~20 s)", "scripts/gui/start_gazebo.sh")

    def run_wipe_mi(self):
        self.script("Section 3 MoveIt wipe (counter, 10 N)", "scripts/gui/wipe_moveit.sh")

    def run_wipe_mi_mirror(self):
        self.script("Section 3 MoveIt wipe (mirror, 6 N)", "scripts/gui/wipe_moveit_mirror.sh")

    def run_wipe(self):
        self.script("Section 3 gz wipe (counter)", "scripts/gui/wipe.sh")

    def run_wipe_mirror(self):
        self.script("Section 3 gz wipe (mirror)", "scripts/gui/wipe_mirror.sh")

    def run_gz_continuous(self):
        self.script("Section 3 Gazebo continuous replay", "scripts/gui/wipe_gz_continuous.sh")

    def loop_gz_continuous(self):
        self.script("Section 3 Gazebo continuous replay loop", "scripts/gui/wipe_gz_continuous.sh --loop")

    def run_all(self):
        self.script("Run ALL sections (headless, ~4-5 min)", "scripts/run_all_jazzy.sh")

    def stop(self):
        self.script("Stop everything", "scripts/gui/stop.sh")

    def show(self, fname):
        path = os.path.join(DATA, fname)
        if not os.path.exists(path):
            self.say(f"  (no {fname} yet — run that section first)")
            return
        self.say(f"  opening {fname}")
        subprocess.Popen(["xdg-open", path])

    def open_outputs(self):
        subprocess.Popen(["xdg-open", os.path.join(REPO, "outputs")])


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
