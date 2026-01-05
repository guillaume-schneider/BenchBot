
import os
import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle, PageBreak
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch

# Reuse trajectory reading logic if possible
from tools.benchmark import read_bag_data, calculate_ate

class SLAMReportGenerator:
    def __init__(self, runs_data, output_path):
        """
        runs_data: list of dicts from BenchmarkPage.parse_run
        output_path: path to save the PDF
        """
        self.runs = runs_data
        self.output_path = output_path
        self.styles = getSampleStyleSheet()
        self.temp_files = []

    def _cleanup(self):
        for f in self.temp_files:
            if os.path.exists(f):
                os.remove(f)

    def generate(self):
        doc = SimpleDocTemplate(self.output_path, pagesize=A4)
        elements = []

        # Title
        title_style = ParagraphStyle(
            'TitleStyle',
            parent=self.styles['Heading1'],
            fontSize=24,
            alignment=1,
            spaceAfter=30
        )
        elements.append(Paragraph("SLAM Bench: Comparison Report", title_style))
        elements.append(Paragraph(f"Generated on: {os.popen('date').read().strip()}", self.styles['Normal']))
        elements.append(Spacer(1, 0.5 * inch))

        # --- PAGE 1: EXECUTIVE SUMMARY ---
        elements.append(Paragraph("1. Executive Summary", self.styles['Heading2']))
        elements.append(Spacer(1, 0.2 * inch))

        # Table data
        table_data = [["Run ID", "SLAM", "ATE (m)", "Coverage", "CPU", "RAM"]]
        for r in self.runs:
            table_data.append([
                r['id'][:15] + "...", 
                r['slam'], 
                f"{r['ate']:.3f}" if r['ate'] else "-",
                f"{r['coverage']:.1f}%" if r['coverage'] else "-",
                f"{r['cpu']:.1f}%" if r['cpu'] else "-",
                f"{r['ram']:.0f}" if r['ram'] else "-"
            ])
        
        t = Table(table_data, colWidths=[1.8*inch, 1*inch, 0.8*inch, 0.8*inch, 0.8*inch, 0.8*inch])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor("#1e293b")),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), colors.beige),
            ('GRID', (0, 0), (-1, -1), 1, colors.black)
        ]))
        elements.append(t)
        elements.append(Spacer(1, 0.5 * inch))

        # Summary Chart: ATE Comparison
        ate_chart = self._plot_bar_chart("ate", "ATE RMSE (Lower is Better)", "m")
        if ate_chart:
            elements.append(Image(ate_chart, width=6*inch, height=3*inch))
            self.temp_files.append(ate_chart)

        elements.append(PageBreak())

        # --- PAGE 2: TRAJECTORY COMPARISON ---
        elements.append(Paragraph("2. Trajectory Comparison", self.styles['Heading2']))
        elements.append(Spacer(1, 0.2 * inch))
        
        traj_plot = self._plot_combined_trajectories()
        if traj_plot:
            elements.append(Image(traj_plot, width=6*inch, height=5*inch))
            self.temp_files.append(traj_plot)
            
        elements.append(Paragraph("Trajectory alignment: Estimates are aligned to Ground Truth at the first pose for better visual comparison of relative drift.", self.styles['Italic']))

        elements.append(PageBreak())

        # --- PAGE 3: RESOURCE USAGE ---
        elements.append(Paragraph("3. Resource Efficiency", self.styles['Heading2']))
        elements.append(Spacer(1, 0.2 * inch))

        cpu_chart = self._plot_bar_chart("cpu", "Peak CPU Usage (Lower is Better)", "%")
        if cpu_chart:
            elements.append(Image(cpu_chart, width=6*inch, height=3.5*inch))
            self.temp_files.append(cpu_chart)

        elements.append(Spacer(1, 0.3 * inch))

        ram_chart = self._plot_bar_chart("ram", "Peak RAM Memory (Lower is Better)", "MB")
        if ram_chart:
            elements.append(Image(ram_chart, width=6*inch, height=3.5*inch))
            self.temp_files.append(ram_chart)

        # Final Build
        doc.build(elements)
        self._cleanup()

    def _plot_bar_chart(self, key, title, unit):
        labels = []
        values = []
        for r in self.runs:
            if r[key] is not None:
                labels.append(f"{r['slam']}\n({r['id'][:8]})")
                values.append(r[key])
        
        if not values: return None
        
        plt.figure(figsize=(10, 5))
        x = np.arange(len(labels))
        bars = plt.bar(x, values, color='#3b82f6', alpha=0.8)
        plt.xticks(x, labels, rotation=0, fontsize=8)
        plt.ylabel(f"{unit}")
        plt.title(title)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        
        # Add values on top
        for bar in bars:
            yval = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2, yval, f"{yval:.2f}", va='bottom', ha='center')

        path = f"/tmp/report_{key}.png"
        plt.tight_layout()
        plt.savefig(path)
        plt.close()
        return path

    def _plot_combined_trajectories(self):
        plt.figure(figsize=(10, 8))
        if not self.runs: return None
        cmap = plt.get_cmap('tab10')
        
        gt_plotted = False
        
        for i, r in enumerate(self.runs):
            # We need to find the bag path
            # Assume results/runs/<id>/bags/output
            bag_path = Path("results/runs") / r['id'] / "bags" / "output"
            if not bag_path.exists(): continue
            
            try:
                gt_poses, tf_data, odom_data = read_bag_data(bag_path)
                if not gt_poses: continue
                
                rmse, errors, gt_traj, est_traj = calculate_ate(gt_poses, tf_data, odom_data)
                
                if gt_traj and not gt_plotted:
                    gt_x = [p[0] for p in gt_traj]
                    gt_y = [p[1] for p in gt_traj]
                    plt.plot(gt_x, gt_y, 'k-', linewidth=2, label='Ground Truth', zorder=1)
                    gt_plotted = True
                
                if est_traj:
                    ex = [p[0] for p in est_traj]
                    ey = [p[1] for p in est_traj]
                    plt.plot(ex, ey, '--', color=cmap(i % 10), label=f"{r['slam']} ({r['id'][:8]})", zorder=2)
            except:
                pass

        plt.title("Combined Trajectories Comparison")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        
        path = "/tmp/report_traj.png"
        plt.savefig(path)
        plt.close()
        return path

