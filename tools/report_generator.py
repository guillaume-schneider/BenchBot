import os
import json
from pathlib import Path
from datetime import datetime
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
from reportlab.lib.units import inch
import matplotlib.pyplot as plt
import tempfile

class SLAMReportGenerator:
    def __init__(self, runs_data, output_path, plot_path=None):
        self.runs_data = runs_data
        self.output_path = output_path
        self.plot_path = plot_path
        self.styles = getSampleStyleSheet()
        # Use landscape for better width handling
        self.doc = SimpleDocTemplate(str(output_path), pagesize=landscape(A4))
        self.elements = []
        
        # Custom styles
        self.styles.add(ParagraphStyle(name='CenterTitle', parent=self.styles['Title'], alignment=1))
        self.styles.add(ParagraphStyle(name='SubTitle', parent=self.styles['Heading2'], color=colors.HexColor("#6366f1")))

    def add_header(self):
        title = Paragraph("SLAM Benchmarking Analysis Report", self.styles['CenterTitle'])
        self.elements.append(title)
        
        date_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        info = Paragraph(f"Generated on: {date_str}", self.styles['Normal'])
        self.elements.append(info)
        self.elements.append(Spacer(1, 0.2 * inch))


    def add_metrics_charts(self):
        if not self.runs_data:
            return

        self.elements.append(PageBreak())
        self.elements.append(Paragraph("Metrics Visualization", self.styles['Title']))

        # Prepare data
        names = []
        for i, r in enumerate(self.runs_data):
            label = r.get('slam', 'Unknown')
            if names.count(label) > 0 or any(x == label for x in names):
                label = f"{label} ({i+1})"
            names.append(label)

        ate = [r.get('ate') if r.get('ate') is not None else 0 for r in self.runs_data]
        
        coverage = [r.get('coverage') if r.get('coverage') is not None else 0 for r in self.runs_data]
        acc_cov = [r.get('accessible_coverage') if r.get('accessible_coverage') is not None else 0 for r in self.runs_data]
        iou = [r.get('occupancy_iou') if r.get('occupancy_iou') is not None else 0 for r in self.runs_data]
        
        cpu = [r.get('cpu') if r.get('cpu') is not None else 0 for r in self.runs_data]
        ram = [r.get('ram') if r.get('ram') is not None else 0 for r in self.runs_data]

        plt.style.use('ggplot')

        # --- Section 1: Trajectory (ATE) ---
        self.elements.append(Paragraph("1. Trajectory Precision", self.styles['Heading2']))
        self.elements.append(Spacer(1, 0.1 * inch))
        
        fig1, ax1 = plt.subplots(figsize=(8, 4))
        ax1.bar(names, ate, color='#ef4444')
        ax1.set_title('ATE RMSE (m) - Lower is Better')
        ax1.tick_params(axis='x', rotation=15)
        plt.tight_layout()
        
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp1:
            plt.savefig(tmp1.name, dpi=120)
            self.elements.append(Image(tmp1.name, width=7*inch, height=3.5*inch))
        plt.close(fig1)
        self.elements.append(Spacer(1, 0.2 * inch))


        # --- Section 2: Map Quality ---
        self.elements.append(Paragraph("2. Map Quality", self.styles['Heading2']))
        self.elements.append(Spacer(1, 0.1 * inch))

        fig2, axs2 = plt.subplots(1, 3, figsize=(12, 4))
        
        axs2[0].bar(names, coverage, color='#22c55e')
        axs2[0].set_title('Global Coverage (%)')
        axs2[0].tick_params(axis='x', rotation=30, labelsize=8)
        
        axs2[1].bar(names, acc_cov, color='#10b981')
        axs2[1].set_title('Accessible Coverage (%)')
        axs2[1].tick_params(axis='x', rotation=30, labelsize=8)
        
        axs2[2].bar(names, iou, color='#8b5cf6')
        axs2[2].set_title('Occupancy IoU (0-1)')
        axs2[2].tick_params(axis='x', rotation=30, labelsize=8)
        
        plt.tight_layout()
        
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp2:
            plt.savefig(tmp2.name, dpi=120)
            self.elements.append(Image(tmp2.name, width=9.5*inch, height=3.2*inch))
        plt.close(fig2)
        self.elements.append(Spacer(1, 0.2 * inch))


        # --- Section 3: System Consumption ---
        self.elements.append(Paragraph("3. System Consumption", self.styles['Heading2']))
        self.elements.append(Spacer(1, 0.1 * inch))

        fig3, axs3 = plt.subplots(1, 2, figsize=(10, 4))
        
        axs3[0].bar(names, cpu, color='#3b82f6')
        axs3[0].set_title('Max CPU (%)')
        axs3[0].tick_params(axis='x', rotation=30, labelsize=8)

        axs3[1].bar(names, ram, color='#f59e0b')
        axs3[1].set_title('Max RAM (MB)')
        axs3[1].tick_params(axis='x', rotation=30, labelsize=8)

        plt.tight_layout()
        
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp3:
            plt.savefig(tmp3.name, dpi=120)
            self.elements.append(Image(tmp3.name, width=8*inch, height=3.2*inch))
        plt.close(fig3)
        self.elements.append(Spacer(1, 0.3 * inch))


    def add_summary_table(self, runs_data):
        self.elements.append(Paragraph("Comparison Summary", self.styles['SubTitle']))
        self.elements.append(Spacer(1, 0.1 * inch))
        
        col_count = len(runs_data)
        

        # If too many runs, switch to Row-based layout (Metrics as columns)
        if col_count > 5:
            # Table Header
            table_header = ["Run ID", "SLAM", "Dataset", "Duration (s)", "ATE (m)", "Cov (%)", "AccCov(%)", "IoU", "Status"]
            data = [table_header]
            
            for i, r in enumerate(runs_data):
                 # ID handling
                run_id = r.get('name', r.get('id', f'Run {i+1}'))
                # Truncate ID if too long
                if len(run_id) > 20: run_id = run_id[:8] + "..." + run_id[-8:]
                
                row = [
                    run_id,
                    r.get('slam', '-'),
                    r.get('dataset', '-'),
                    f"{r.get('duration', 0):.2f}" if r.get('duration') is not None else "-",
                    f"{r.get('ate', 0):.3f}" if r.get('ate') is not None else "-",
                    f"{r.get('coverage', 0):.1f}" if r.get('coverage') is not None else "-",
                    f"{r.get('accessible_coverage', 0):.1f}" if r.get('accessible_coverage') is not None else "-",
                    f"{r.get('occupancy_iou', 0):.3f}" if r.get('occupancy_iou') is not None else "-",
                    r.get('status', '-')
                ]
                data.append(row)
                
            col_widths = [1.8*inch, 1.1*inch, 1.4*inch, 0.9*inch, 0.7*inch, 0.7*inch, 0.8*inch, 0.6*inch, 0.8*inch]
            t = Table(data, hAlign='LEFT', colWidths=col_widths)
            
        else:
            # Pivot Layout (Comparison style) for few runs
            header = ["Metric", "Run 1", "Run 2", "Run 3"]
            table_header = ["Metric"] + [f"Run {i+1}" for i in range(col_count)]
            
            # Helper for formatting
            def fmt(key, val):
                if val is None or val == "-": return "-"
                try:
                    v = float(val)
                    if key in ["ate", "occupancy_iou", "ssim", "lidar_noise"]: return f"{v:.4f}"
                    if key in ["coverage", "accessible_coverage", "cpu", "ram", "lidar_range"]: return f"{v:.1f}"
                    if key in ["wall_thick", "duration"]: return f"{v:.2f}"
                    if key in ["speed_scale"]: return f"{v:.0f}"
                except: pass
                return str(val)

            data = [
                table_header,
                ["Health Status"] + [r.get('status', '-') for r in runs_data],
                ["SLAM Algorithm"] + [r.get('slam', '-') for r in runs_data],
                ["Dataset"] + [r.get('dataset', '-') for r in runs_data],
                ["Duration (s)"] + [fmt('duration', r.get('duration')) for r in runs_data],
                ["ATE RMSE (m)"] + [fmt('ate', r.get('ate')) for r in runs_data],
                ["Coverage (%)"] + [fmt('coverage', r.get('coverage')) for r in runs_data],
                ["Acc. Coverage (%)"] + [fmt('accessible_coverage', r.get('accessible_coverage')) for r in runs_data],
                ["Occupancy IoU"] + [fmt('occupancy_iou', r.get('occupancy_iou')) for r in runs_data],
                ["Structural Similarity"] + [fmt('ssim', r.get('ssim')) for r in runs_data],
                ["Wall Thick. (cm)"] + [fmt('wall_thick', r.get('wall_thick')) for r in runs_data],
                ["Max RAM (MB)"] + [fmt('ram', r.get('ram')) for r in runs_data],
                ["Max CPU (%)"] + [fmt('cpu', r.get('cpu')) for r in runs_data],
                ["Lidar Noise (std)"] + [fmt('lidar_noise', r.get('lidar_noise')) for r in runs_data],
                ["Max Range (m)"] + [fmt('lidar_range', r.get('lidar_range')) for r in runs_data],
                ["Speed Scale (%)"] + [fmt('speed_scale', r.get('speed_scale')) for r in runs_data],
            ]
            t = Table(data, hAlign='LEFT')

        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor("#1e293b")),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), colors.whitesmoke),
            ('GRID', (0, 0), (-1, -1), 1, colors.grey),
        ]))
        
        self.elements.append(t)
        self.elements.append(Spacer(1, 0.3 * inch))

    def add_plot(self, plot_path):
        if os.path.exists(plot_path):
            self.elements.append(Paragraph("Trajectory Visualization", self.styles['SubTitle']))
            self.elements.append(Spacer(1, 0.1 * inch))
            # Adjust image size for landscape
            img = Image(plot_path, width=8*inch, height=5*inch)
            self.elements.append(img)
            self.elements.append(Spacer(1, 0.2 * inch))

    def add_map_comparison(self, runs_data):
        self.elements.append(PageBreak())
        self.elements.append(Paragraph("Map Reconstruction Comparison", self.styles['SubTitle']))
        self.elements.append(Spacer(1, 0.2 * inch))
        
        # Display GT Map (Use the first one found)
        gt_path = None
        for r in runs_data:
            if r.get('gt_map_image_path') and os.path.exists(r.get('gt_map_image_path')):
                gt_path = r.get('gt_map_image_path')
                break
        
        if gt_path:
            self.elements.append(Paragraph("Ground Truth Map", self.styles['Heading3']))
            self.elements.append(Image(gt_path, width=4*inch, height=4*inch, kind='proportional'))
            self.elements.append(Spacer(1, 0.2 * inch))
            
        # Display Generated Maps in a grid (2 per row)
        self.elements.append(Paragraph("Generated Maps", self.styles['Heading3']))
        self.elements.append(Spacer(1, 0.1 * inch))
        
        map_images = []
        for i, r in enumerate(runs_data):
            path = r.get('map_image_path')
            title = f"{r.get('slam', 'Run '+str(i+1))} (IoU: {r.get('occupancy_iou', 0):.2f})"
            if path and os.path.exists(path):
                img = Image(path, width=3.5*inch, height=3.5*inch, kind='proportional')
                map_images.append([img, Paragraph(title, self.styles['Normal'])])
            else:
                map_images.append([Paragraph("Map Image Not Found", self.styles['Normal']), Paragraph(title, self.styles['Normal'])])

        # Batch into rows of 3 for Landscape
        row_data = []
        current_row = []
        for item in map_images:
            # Create a mini table for Image + Title
            t_cell = Table([[item[0]], [item[1]]], colWidths=[3.6*inch], rowHeights=[3.6*inch, 0.4*inch])
            t_cell.setStyle(TableStyle([('ALIGN', (0,0), (-1,-1), 'CENTER'), ('VALIGN', (0,0), (-1,-1), 'MIDDLE')]))
            current_row.append(t_cell)
            
            if len(current_row) == 3:
                row_data.append(current_row)
                current_row = []
        
        if current_row:
            # Pad with empty cells
            while len(current_row) < 3:
                current_row.append("")
            row_data.append(current_row)
            
        if row_data:
            t = Table(row_data, colWidths=[3.8*inch, 3.8*inch, 3.8*inch])
            t.setStyle(TableStyle([
                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                ('VALIGN', (0,0), (-1,-1), 'TOP'),
            ]))
            self.elements.append(t)


    def add_metric_explanations(self):
        self.elements.append(Paragraph("Guide to Metrics", self.styles['Heading2']))
        self.elements.append(Spacer(1, 0.1 * inch))
        
        definitions = [
            ("<b>ATE RMSE (Absolute Trajectory Error)</b>: Measures the global consistency of the trajectory. Lower is better. Values < 0.1m indicate high precision; > 0.5m suggest significant drift or SLAM failure.", 
             "#ef4444"), # Red
            ("<b>Coverage (%)</b>: Percentage of the Ground Truth free space that was successfully mapped as free or occupied. Higher is better.", 
             "#22c55e"), # Green
            ("<b>Acc. Coverage (%)</b>: Coverage restricted to the area actually visited/approached by the robot. Useful to judge exploration efficiency independent of mission completeness.", 
             "#22c55e"),
            ("<b>Occupancy IoU (Intersection over Union)</b>: Measures how well the estimated obstacles match the Ground Truth walls. Ranges 0 to 1. 1.0 is perfect alignment. < 0.5 usually implies map distortion or offset.", 
             "#6366f1"), # Indigo
            ("<b>SSIM (Structural Similarity)</b>: Visual similarity between generated map and Ground Truth. 1.0 is identical. Captures general structure better than pixel-wise metrics.", 
             "#8b5cf6"), # Purple
            ("<b>Wall Thickness</b>: Average thickness of mapped walls. Thicker walls than reality (e.g. > 15cm) indicate 'blur' or uncertainty in the map.", 
             "#f59e0b"), # Amber
            ("<b>Duration (s)</b>: Total time to map the environment. Lower is better for efficiency, assuming map quality is maintained.", 
             "#64748b"), # Slate
            ("<b>Max CPU (%)</b>: Peak processor usage during the run. Lower indicates better computational efficiency, critical for onboard operations.", 
             "#ef4444"), # Red
            ("<b>Max RAM (MB)</b>: Peak memory usage. Lower is better, especially for constrained hardware platforms.", 
             "#ef4444")  # Red
        ]
        
        for text, color in definitions:
            p = Paragraph(text, self.styles['Normal'])
            p.textColor = colors.HexColor(color)
            self.elements.append(p)
            self.elements.append(Spacer(1, 0.05 * inch))
            
        self.elements.append(Spacer(1, 0.2 * inch))

    def add_run_details(self, runs_data):
        for i, run in enumerate(runs_data):
            self.elements.append(PageBreak())
            self.elements.append(Paragraph(f"Detailed Analysis: Run {i+1}", self.styles['SubTitle']))
            run_id = run.get('name', run.get('id', 'Unknown'))
            self.elements.append(Paragraph(f"ID: {run_id}", self.styles['Code']))
            self.elements.append(Spacer(1, 0.15 * inch))
            
            # --- Automated Interpretation ---
            slam = run.get('slam', 'Unknown SLAM')
            ate = run.get('ate')
            cov = run.get('coverage')
            iou = run.get('occupancy_iou')
            
            analysis_text = f"<b>Performance Summary:</b><br/>"
            analysis_text += f"The algorithm <b>{slam}</b> "
            
            # ATE Analysis
            if ate is not None:
                if ate < 0.1: analysis_text += "demonstrated <b>excellent localization accuracy</b> with negligible drift. "
                elif ate < 0.5: analysis_text += "showed <b>acceptable accuracy</b>, though some minor drift occurred. "
                else: analysis_text += "suffered from <b>significant trajectory drift</b>, suggesting localization failure. "
            
            # Coverage Analysis
            if cov is not None:
                if cov > 90: analysis_text += "Exploration was <b>highly complete</b>, covering most of the environment. "
                elif cov > 50: analysis_text += "Exploration was <b>partial</b>, leaving significant areas unmapped. "
                else: analysis_text += "Exploration was <b>minimal</b>. "

            # IoU Analysis
            if iou is not None:
                if iou > 0.7: analysis_text += "The resulting map structure is <b>highly accurate</b> aligned with Ground Truth. "
                elif iou > 0.4: analysis_text += "The map captures the general layout but contains <b>noticeable distortions</b> or ghost obstacles. "
                else: analysis_text += "The map quality is <b>poor</b>, with little overlap with the actual environment. "
                
            p = Paragraph(analysis_text, self.styles['Normal'])
            self.elements.append(p)
            self.elements.append(Spacer(1, 0.1 * inch))
            # -------------------------------
            
            if run.get('reasons'):
                self.elements.append(Spacer(1, 0.1 * inch))
                self.elements.append(Paragraph("Detected Anomalies:", self.styles['Heading4']))
                for reason in run['reasons']:
                    self.elements.append(Paragraph(f"• {reason}", self.styles['Normal']))

    def generate(self):
        self.add_header()
        self.add_summary_table(self.runs_data)
        self.add_metric_explanations() # Added Guide
        self.add_metrics_charts()
        if self.plot_path:
            self.add_plot(self.plot_path)
        self.add_map_comparison(self.runs_data)
        self.add_run_details(self.runs_data)
        self.doc.build(self.elements)
        return self.output_path

def generate_full_report(output_file, runs_data, plot_path=None):
    gen = SLAMReportGenerator(runs_data, output_file, plot_path)
    return gen.generate()
