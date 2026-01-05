import os
import json
from pathlib import Path
from datetime import datetime
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
from reportlab.lib.units import inch

class ReportGenerator:
    def __init__(self, output_path):
        self.output_path = output_path
        self.styles = getSampleStyleSheet()
        self.doc = SimpleDocTemplate(str(output_path), pagesize=A4)
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

    def add_summary_table(self, runs_data):
        """
        runs_data: list of dicts {name, slam, dataset, ate, coverage, ram, cpu, status}
        """
        self.elements.append(Paragraph("Comparison Summary", self.styles['SubTitle']))
        self.elements.append(Spacer(1, 0.1 * inch))
        
        header = ["Metric", "Run 1", "Run 2", "Run 3"]
        # Filter out "Run X" if no data
        col_count = len(runs_data)
        table_header = ["Metric"] + [f"Run {i+1}" for i in range(col_count)]
        
        data = [
            table_header,
            ["Health Status"] + [r.get('status', '-') for r in runs_data],
            ["SLAM Algorithm"] + [r.get('slam', '-') for r in runs_data],
            ["Dataset"] + [r.get('dataset', '-') for r in runs_data],
            ["ATE RMSE (m)"] + [r.get('ate', '-') for r in runs_data],
            ["Coverage (%)"] + [r.get('coverage', '-') for r in runs_data],
            ["Structural Similarity"] + [r.get('ssim', '-') for r in runs_data],
            ["Wall Thick. (cm)"] + [r.get('wall_thick', '-') for r in runs_data],
            ["Max RAM (MB)"] + [r.get('ram', '-') for r in runs_data],
            ["Max CPU (%)"] + [r.get('cpu', '-') for r in runs_data],
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
            # Color status cells
            *[('TEXTCOLOR', (i+1, 1), (i+1, 1), colors.red if runs_data[i].get('is_failure') else colors.green) for i in range(col_count)]
        ]))
        self.elements.append(t)
        self.elements.append(Spacer(1, 0.3 * inch))

    def add_plot(self, plot_path):
        if os.path.exists(plot_path):
            self.elements.append(Paragraph("Trajectory Visualization", self.styles['SubTitle']))
            self.elements.append(Spacer(1, 0.1 * inch))
            img = Image(plot_path, width=6*inch, height=4*inch)
            self.elements.append(img)
            self.elements.append(Spacer(1, 0.2 * inch))

    def add_run_details(self, runs_data):
        for i, run in enumerate(runs_data):
            self.elements.append(PageBreak())
            self.elements.append(Paragraph(f"Detailed Analysis: Run {i+1}", self.styles['SubTitle']))
            self.elements.append(Paragraph(f"ID: {run['name']}", self.styles['Code']))
            
            if run.get('reasons'):
                self.elements.append(Spacer(1, 0.1 * inch))
                self.elements.append(Paragraph("Detected Anomalies:", self.styles['Heading4']))
                for reason in run['reasons']:
                    self.elements.append(Paragraph(f"• {reason}", self.styles['Normal']))

    def generate(self):
        self.doc.build(self.elements)
        return self.output_path

def generate_full_report(output_file, runs_data, plot_path=None):
    gen = ReportGenerator(output_file)
    gen.add_header()
    gen.add_summary_table(runs_data)
    if plot_path:
        gen.add_plot(plot_path)
    gen.add_run_details(runs_data)
    return gen.generate()
