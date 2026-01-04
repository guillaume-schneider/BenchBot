import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from .generator import generate_map
from .viewer import show_map
import os

class GTMapGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Ground Truth Map Generator")
        self.root.geometry("600x450")
        
        # Main container
        main_frame = ttk.Frame(root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # File selection
        ttk.Label(main_frame, text="Gazebo Model (SDF):", font=('Helvetica', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=(0, 5))
        self.sdf_path = tk.StringVar(value="world/model.sdf")
        ttk.Entry(main_frame, textvariable=self.sdf_path, width=50).grid(row=1, column=0, padx=(0, 10))
        ttk.Button(main_frame, text="Browse", command=self.browse_sdf).grid(row=1, column=1)

        # Parameters
        param_frame = ttk.LabelFrame(main_frame, text="Parameters", padding="10")
        param_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=20)

        # Resolution
        ttk.Label(param_frame, text="Resolution (m/px):").grid(row=0, column=0, sticky=tk.W)
        self.res = tk.DoubleVar(value=0.05)
        ttk.Entry(param_frame, textvariable=self.res, width=10).grid(row=0, column=1, sticky=tk.W, padx=10, pady=5)

        # Laser Z
        ttk.Label(param_frame, text="Laser Height (m):").grid(row=1, column=0, sticky=tk.W)
        self.laser_z = tk.DoubleVar(value=0.17)
        ttk.Entry(param_frame, textvariable=self.laser_z, width=10).grid(row=1, column=1, sticky=tk.W, padx=10, pady=5)

        # Padding
        ttk.Label(param_frame, text="Padding (m):").grid(row=2, column=0, sticky=tk.W)
        self.padding = tk.DoubleVar(value=1.5)
        ttk.Entry(param_frame, textvariable=self.padding, width=10).grid(row=2, column=1, sticky=tk.W, padx=10, pady=5)

        # Output Name
        ttk.Label(param_frame, text="Output Base Name:").grid(row=3, column=0, sticky=tk.W)
        self.out_name = tk.StringVar(value="map_gt")
        ttk.Entry(param_frame, textvariable=self.out_name, width=20).grid(row=3, column=1, sticky=tk.W, padx=10, pady=5)

        # Options
        option_frame = ttk.LabelFrame(main_frame, text="Options", padding="5")
        option_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.gen_png = tk.BooleanVar(value=True)
        ttk.Checkbutton(option_frame, text="Generate PNG image", variable=self.gen_png).grid(row=0, column=0, sticky=tk.W, padx=10)
        
        self.gen_debug = tk.BooleanVar(value=True)
        ttk.Checkbutton(option_frame, text="Generate Debug geometry plot", variable=self.gen_debug).grid(row=0, column=1, sticky=tk.W, padx=10)

        # Action Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.grid(row=5, column=0, columnspan=2, pady=10)

        self.generate_btn = ttk.Button(btn_frame, text="Generate GT Map", command=self.run_generation)
        self.generate_btn.pack(side=tk.LEFT, padx=5)

        self.show_btn = ttk.Button(btn_frame, text="Preview Result", command=self.preview_map)
        self.show_btn.pack(side=tk.LEFT, padx=5)
        
        self.show_debug_btn = ttk.Button(btn_frame, text="Show Debug Plot", command=self.preview_debug)
        self.show_debug_btn.pack(side=tk.LEFT, padx=5)

        # Status
        self.status = tk.StringVar(value="Ready")
        self.status_label = ttk.Label(main_frame, textvariable=self.status, foreground="blue", wraplength=500)
        self.status_label.grid(row=6, column=0, columnspan=2, pady=5)

    def browse_sdf(self):
        filename = filedialog.askopenfilename(filetypes=[("SDF files", "*.sdf"), ("All files", "*.*")])
        if filename:
            self.sdf_path.set(filename)

    def run_generation(self):
        self.status.set("Generating...")
        self.root.update_idletasks()
        
        success, msg = generate_map(
            sdf_path=self.sdf_path.get(),
            resolution=self.res.get(),
            laser_z=self.laser_z.get(),
            padding=self.padding.get(),
            output_name=self.out_name.get(),
            gen_png=self.gen_png.get(),
            gen_debug=self.gen_debug.get()
        )
        
        if success:
            self.status.set("Success!")
            messagebox.showinfo("Success", msg)
        else:
            self.status.set("Error occurred")
            messagebox.showerror("Error", msg)

    def preview_map(self):
        yaml_path = f"{self.out_name.get()}.yaml"
        if os.path.exists(yaml_path):
            try:
                show_map(yaml_path)
            except Exception as e:
                messagebox.showerror("Visualization Error", str(e))
        else:
            messagebox.showwarning("Warning", f"Map file {yaml_path} not found. Generate it first.")

    def preview_debug(self):
        debug_path = f"{self.out_name.get()}_debug.png"
        if os.path.exists(debug_path):
            try:
                import matplotlib.pyplot as plt
                import matplotlib.image as mpimg
                img = mpimg.imread(debug_path)
                plt.figure("Debug Geometry Plot", figsize=(10, 10))
                plt.imshow(img)
                plt.axis('off')
                plt.show()
            except Exception as e:
                messagebox.showerror("Visualization Error", str(e))
        else:
            messagebox.showwarning("Warning", f"Debug plot {debug_path} not found. Generate it first.")

if __name__ == "__main__":
    root = tk.Tk()
    app = GTMapGUI(root)
    root.mainloop()
