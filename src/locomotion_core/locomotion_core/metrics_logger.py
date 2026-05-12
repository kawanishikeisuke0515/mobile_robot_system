"""
metrics_logger.py
─────────────────────────────────────────────────────────────────────────────
Standalone metrics logger for the GmrTracker node.

Produces two CSV files per run (timestamped so runs don't overwrite each other):

  gmr_tracking_<stamp>.csv   — per-tick tracking data
    columns: wall_time, index, x, y, x_ref, y_ref, tracking_error

  gmr_replan_<stamp>.csv     — one row per replan event
    columns: wall_time, index, n_obstacles, gmr_ms, avoid_ms, total_replan_ms

  gmr_summary_<stamp>.txt    — human-readable summary printed + saved on completion

Usage inside GmrTracker
───────────────────────
  from metrics_logger import MetricsLogger
  import time

  # In __init__:
  self.metrics = MetricsLogger(output_dir="/tmp/gmr_metrics")

  # In control_callback, after computing errors:
  self.metrics.log_tick(self.current_index, self.x, self.y,
                        x_d, y_d, math.hypot(ex_w, ey_w))

  # In _replan(), wrapping the timed sections:
  t0 = time.perf_counter()
  gmr_mean, _ = gmr(...)
  t1 = time.perf_counter()
  avoided_xy  = avoid_obstacles_2d(...)
  t2 = time.perf_counter()
  self.metrics.log_replan(self.current_index, len(obstacles),
                          gmr_ms   = (t1 - t0) * 1e3,
                          avoid_ms = (t2 - t1) * 1e3)

  # At the end of control_callback when trajectory completes:
  self.metrics.finalize()
"""

import csv
import math
import os
import time
from datetime import datetime


class MetricsLogger:
    """
    Collects and persists metrics for one GMR tracker run.

    Parameters
    ----------
    output_dir : str
        Directory where CSV / summary files are written.
        Created automatically if it does not exist.
    """

    def __init__(self, output_dir: str = "/tmp/gmr_metrics"):
        os.makedirs(output_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._track_path   = os.path.join(output_dir, f"gmr_tracking_{stamp}.csv")
        self._replan_path  = os.path.join(output_dir, f"gmr_replan_{stamp}.csv")
        self._summary_path = os.path.join(output_dir, f"gmr_summary_{stamp}.txt")

        # ── open files and write headers ──────────────────────────
        self._track_fh  = open(self._track_path,  "w", newline="")
        self._replan_fh = open(self._replan_path, "w", newline="")

        self._track_writer = csv.writer(self._track_fh)
        self._track_writer.writerow(
            ["wall_time", "index", "x", "y", "x_ref", "y_ref", "tracking_error"]
        )

        self._replan_writer = csv.writer(self._replan_fh)
        self._replan_writer.writerow(
            ["wall_time", "index", "n_obstacles", "gmr_ms", "avoid_ms", "total_replan_ms"]
        )

        # ── internal state ─────────────────────────────────────────
        self._t_start       = time.perf_counter()
        self._tick_errors   = []          # tracking_error per tick
        self._replan_count  = 0
        self._finalized     = False

        print(f"[MetricsLogger] Logging to:\n"
              f"  tracking : {self._track_path}\n"
              f"  replans  : {self._replan_path}\n"
              f"  summary  : {self._summary_path}")

    # ──────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────

    def log_tick(self,
                 index: int,
                 x: float, y: float,
                 x_ref: float, y_ref: float,
                 tracking_error: float) -> None:
        """
        Call once per control tick.

        Parameters
        ----------
        index          : current trajectory index
        x, y           : actual robot position
        x_ref, y_ref   : reference position at this index
        tracking_error : Euclidean distance ||(x,y) - (x_ref,y_ref)||
        """
        if self._finalized:
            return

        wall = time.perf_counter() - self._t_start
        self._track_writer.writerow(
            [f"{wall:.4f}", index,
             f"{x:.4f}", f"{y:.4f}",
             f"{x_ref:.4f}", f"{y_ref:.4f}",
             f"{tracking_error:.4f}"]
        )
        self._tick_errors.append(tracking_error)

    def log_replan(self,
                   index: int,
                   n_obstacles: int,
                   gmr_ms: float,
                   avoid_ms: float) -> None:
        """
        Call once per _replan() invocation, after timing both stages.

        Parameters
        ----------
        index       : trajectory index at time of replan
        n_obstacles : number of active obstacles
        gmr_ms      : GMR inference wall time in milliseconds
        avoid_ms    : avoid_obstacles_2d wall time in milliseconds
        """
        if self._finalized:
            return

        wall       = time.perf_counter() - self._t_start
        total_ms   = gmr_ms + avoid_ms
        self._replan_writer.writerow(
            [f"{wall:.4f}", index, n_obstacles,
             f"{gmr_ms:.3f}", f"{avoid_ms:.3f}", f"{total_ms:.3f}"]
        )
        self._replan_count += 1

    def finalize(self) -> None:
        """
        Call once when the trajectory is complete (or on node shutdown).
        Flushes files, computes summary statistics, and prints + saves them.
        """
        if self._finalized:
            return
        self._finalized = True

        total_wall = time.perf_counter() - self._t_start

        # flush & close CSV files
        self._track_fh.flush()
        self._replan_fh.flush()
        self._track_fh.close()
        self._replan_fh.close()

        # ── compute summary stats ──────────────────────────────────
        errors = self._tick_errors
        n      = len(errors)

        if n > 0:
            mean_err = sum(errors) / n
            max_err  = max(errors)
            # root-mean-square error
            rmse     = math.sqrt(sum(e * e for e in errors) / n)
            # fraction of ticks within 5 cm
            pct_5cm  = 100.0 * sum(1 for e in errors if e < 0.05) / n
        else:
            mean_err = max_err = rmse = pct_5cm = float("nan")

        # ── read replan timing for summary ─────────────────────────
        gmr_times   = []
        avoid_times = []
        total_times = []
        try:
            with open(self._replan_path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    gmr_times.append(float(row["gmr_ms"]))
                    avoid_times.append(float(row["avoid_ms"]))
                    total_times.append(float(row["total_replan_ms"]))
        except Exception:
            pass

        def _stats(vals):
            if not vals:
                return "n/a"
            return (f"mean={sum(vals)/len(vals):.2f} ms  "
                    f"max={max(vals):.2f} ms  "
                    f"min={min(vals):.2f} ms")

        lines = [
            "═" * 60,
            "  GMR TRACKER — RUN SUMMARY",
            "═" * 60,
            f"  Total wall-clock time  : {total_wall:.2f} s",
            f"  Control ticks logged   : {n}",
            f"  Replan events          : {self._replan_count}",
            "",
            "  Tracking error (metres)",
            f"    Mean   : {mean_err:.4f} m",
            f"    Max    : {max_err:.4f} m",
            f"    RMSE   : {rmse:.4f} m",
            f"    < 5 cm : {pct_5cm:.1f} % of ticks",
            "",
            "  Replan timing",
            f"    GMR inference  : {_stats(gmr_times)}",
            f"    Obstacle avoid : {_stats(avoid_times)}",
            f"    Total replan   : {_stats(total_times)}",
            "",
            f"  Files",
            f"    {self._track_path}",
            f"    {self._replan_path}",
            f"    {self._summary_path}",
            "═" * 60,
        ]

        summary_text = "\n".join(lines)
        print(summary_text)

        with open(self._summary_path, "w") as f:
            f.write(summary_text + "\n")
