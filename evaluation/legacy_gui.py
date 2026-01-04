#!/usr/bin/env python3

import sys
import os
import tkinter as tk
from tkinter import filedialog, scrolledtext

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import numpy as np

# === import your evaluation functions from metrics.py ===
from .metrics import (
    load_gt_map,
    read_messages_by_topic,
    occupancy_arrays_from_msgs,
    compute_coverage,
    compute_iou,
    compute_time_to_coverage,
    compute_path_length,
)


class BenchmarkApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("ROS2 Exploration Benchmark (Tk GUI)")
        self.geometry("1100x800")

        self.bag_path = None
        self.gt_map_path = None

        # ---------- Top: file selectors ----------
        top_frame = tk.Frame(self)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        btn_bag = tk.Button(top_frame, text="Select ros2 bag folder", command=self.select_bag)
        btn_bag.pack(side=tk.LEFT, padx=5)

        btn_gt = tk.Button(top_frame, text="Select GT map (YAML)", command=self.select_gt)
        btn_gt.pack(side=tk.LEFT, padx=5)

        btn_run = tk.Button(top_frame, text="Run benchmark", command=self.run_benchmark)
        btn_run.pack(side=tk.LEFT, padx=10)

        # ---------- Middle: results text ----------
        self.text = scrolledtext.ScrolledText(self, height=10)
        self.text.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        # ---------- Bottom: matplotlib figure (maps) ----------
        fig, axes = plt.subplots(1, 2, figsize=(8, 4))
        self.fig = fig
        self.ax_gt = axes[0]
        self.ax_est = axes[1]

        self.ax_gt.set_title("Ground Truth")
        self.ax_est.set_title("Estimated")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

        # initial blank display
        self.ax_gt.imshow(np.zeros((10, 10)), cmap="gray", vmin=0, vmax=255)
        self.ax_est.imshow(np.zeros((10, 10)), cmap="gray", vmin=0, vmax=255)
        self.canvas.draw()

    # ------------------- file selection -------------------

    def select_bag(self):
        folder = filedialog.askdirectory(title="Select ros2 bag folder")
        if folder:
            self.bag_path = folder
            self.log(f"Selected bag: {folder}")

    def select_gt(self):
        path = filedialog.askopenfilename(
            title="Select GT YAML map",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if path:
            self.gt_map_path = path
            self.log(f"Selected GT map: {path}")

    # ------------------- logging helper -------------------

    def log(self, msg: str):
        self.text.insert(tk.END, msg + "\n")
        self.text.see(tk.END)

    # ------------------- main benchmark logic -------------------

    def run_benchmark(self):
        if not self.bag_path or not self.gt_map_path:
            self.log("❗ Please select BOTH a ros2 bag folder and a GT map YAML first.")
            return

        try:
            self.log("\n=== Running benchmark ===")

            # 1) Load GT map
            gt_map, gt_res, gt_origin = load_gt_map(self.gt_map_path)
            self.log(f"Loaded GT map: shape={gt_map.shape}, res={gt_res}, origin={gt_origin}")

            # 2) Read bag topics
            topics = ["/map", "/odom"]
            msgs_by_topic = read_messages_by_topic(self.bag_path, topics)

            map_msgs = msgs_by_topic.get("/map", [])
            odom_msgs = msgs_by_topic.get("/odom", [])

            if not map_msgs:
                self.log("❌ No /map messages found in bag.")
                return
            if not odom_msgs:
                self.log("⚠ No /odom messages found; path length will be 0.")

            # 3) Metrics
            est_map = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
            final_cov = compute_coverage(gt_map, est_map)
            final_iou = compute_iou(gt_map, est_map)
            times_to_cov = compute_time_to_coverage(
                gt_map, gt_res, gt_origin, map_msgs, [0.5, 0.8, 0.9]
            )
            path_len = compute_path_length(odom_msgs)

            # 4) Log results
            self.log("\n--- Results ---")
            self.log(f"Final coverage      : {final_cov*100:.2f} %")
            self.log(f"Final occupancy IoU : {final_iou:.4f}")
            self.log(f"Total path length   : {path_len:.2f} m")

            t0 = map_msgs[0][0]
            for th, t in times_to_cov.items():
                if t is None:
                    self.log(f"Time to {int(th*100)}% coverage: not reached")
                else:
                    self.log(f"Time to {int(th*100)}% coverage: {t - t0:.2f} s")

            # 5) Update map images in the figure
            self.update_maps(gt_map, est_map)

            self.log("\n✅ Benchmark done.\n")

        except Exception as e:
            self.log(f"❌ ERROR: {e}")

    # ------------------- map visualization -------------------

    def _to_vis_image(self, occ_grid: np.ndarray):
        """Convert occupancy grid (-1, 0..100) to 0..255 grayscale for display."""
        img = occ_grid.copy().astype(np.int16)

        vis = np.zeros_like(img, dtype=np.uint8)
        vis[img == -1] = 128          # unknown -> mid gray
        vis[img == 0] = 255           # free -> white
        vis[img > 50] = 0             # occupied -> black

        return vis

    def update_maps(self, gt_map, est_map):
        gt_vis = self._to_vis_image(gt_map)
        est_vis = self._to_vis_image(est_map)

        self.ax_gt.clear()
        self.ax_est.clear()

        self.ax_gt.set_title("Ground Truth")
        self.ax_est.set_title("Estimated")

        self.ax_gt.imshow(gt_vis, cmap="gray", vmin=0, vmax=255)
        self.ax_est.imshow(est_vis, cmap="gray", vmin=0, vmax=255)

        self.ax_gt.axis("off")
        self.ax_est.axis("off")

        self.canvas.draw_idle()


if __name__ == "__main__":
    app = BenchmarkApp()
    app.mainloop()
