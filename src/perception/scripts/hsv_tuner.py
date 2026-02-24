#!/home/willjohnson/miniconda3/envs/tensegrity/bin/python
import cv2
import numpy as np
import os
import sys
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import rospkg
import json

class HSVFilterApp:
    def __init__(self, root, directory):
        self.root = root
        self.root.title("HSV Filter Tuner")

        self.package_path = rospkg.RosPack().get_path('tensegrity_perception')
        self.directory = os.path.join(self.package_path,'../../data',directory)
        if not os.path.isdir(self.directory):
            print(f"Directory {self.directory} does not exist")
            sys.exit(1)
        self.frame_idx = 0
        self.image_file = f"{self.directory}/color/{self.frame_idx:04d}.png"

        self.min_hue_r = 160
        self.max_hue_r = 180
        self.min_sat_r = 0
        self.max_sat_r = 255
        self.min_val_r = 0
        self.max_val_r = 255

        self.min_hue_g = 75
        self.max_hue_g = 95
        self.min_sat_g = 0
        self.max_sat_g = 255
        self.min_val_g = 0
        self.max_val_g = 255

        self.min_hue_b = 100
        self.max_hue_b = 120
        self.min_sat_b = 0
        self.max_sat_b = 255
        self.min_val_b = 0
        self.max_val_b = 255

        self.min_hue_dict = {"r": self.min_hue_r, "g": self.min_hue_g, "b": self.min_hue_b}
        self.max_hue_dict = {"r": self.max_hue_r, "g": self.max_hue_g, "b": self.max_hue_b}

        self.create_widgets()
        self.configure_grid()
        self.update_image()

    def create_widgets(self):
        self.root.bind('<Left>', lambda event: self.prev_frame())
        self.root.bind('<Right>', lambda event: self.next_frame())

        # save button
        self.btn_save = ttk.Button(self.root, text="Save HSV", command=self.save_hsv_values)
        self.btn_save.grid(row=9, column=1, columnspan=2, sticky="ew")

        self.label_directory = ttk.Label(self.root, text=f"Directory: {self.directory}", font=("Helvetica", 12))
        self.label_directory.grid(row=0, column=0, columnspan=4, sticky="ew")

        self.canvas_orig = tk.Canvas(self.root, width=400, height=300)
        self.canvas_filtered = tk.Canvas(self.root, width=400, height=300)
        self.canvas_orig.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self.canvas_filtered.grid(row=1, column=2, columnspan=2, sticky="nsew")

        self.create_slider_group("Red", 2, "r")
        self.create_slider_group("Green", 4, "g")
        self.create_slider_group("Blue", 6, "b")

        self.btn_prev = ttk.Button(self.root, text="<", command=self.prev_frame)
        self.btn_prev.grid(row=8, column=0, sticky="ew")
        self.label_image = ttk.Label(self.root, text="Image: 0000.png", font=("Helvetica", 12), anchor="center")
        self.label_image.grid(row=8, column=1, columnspan=2, sticky="ew")
        self.btn_next = ttk.Button(self.root, text=">", command=self.next_frame)
        self.btn_next.grid(row=8, column=3, sticky="ew")

    def create_slider_group(self, color_name, row_start, color_code):
        frame = tk.LabelFrame(self.root, text=color_name, padx=5, pady=5)
        frame.grid(row=row_start, column=0, columnspan=4, pady=5, padx=5, sticky="ew")

        ttk.Label(frame, text="H:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, text="S:").grid(row=1, column=0, sticky="w")
        ttk.Label(frame, text="V:").grid(row=2, column=0, sticky="w")

        min_hue_init = self.min_hue_dict[color_code]
        max_hue_init = self.max_hue_dict[color_code]

        min_hue_slider, min_hue_entry = self.create_slider_with_entry(frame, "Min Hue", min_hue_init, self.update_image)
        max_hue_slider, max_hue_entry = self.create_slider_with_entry(frame, "Max Hue", max_hue_init, self.update_image)
        min_sat_slider, min_sat_entry = self.create_slider_with_entry(frame, "Min Saturation", self.min_sat_r, self.update_image)
        max_sat_slider, max_sat_entry = self.create_slider_with_entry(frame, "Max Saturation", self.max_sat_r, self.update_image)
        min_val_slider, min_val_entry = self.create_slider_with_entry(frame, "Min Value", self.min_val_r, self.update_image)
        max_val_slider, max_val_entry = self.create_slider_with_entry(frame, "Max Value", self.max_val_r, self.update_image)

        min_hue_entry.grid(row=0, column=1, sticky="w")
        min_hue_slider.grid(row=0, column=2, sticky="ew")
        max_hue_slider.grid(row=0, column=3, sticky="ew")
        max_hue_entry.grid(row=0, column=4, sticky="e")

        min_sat_entry.grid(row=1, column=1, sticky="w")
        min_sat_slider.grid(row=1, column=2, sticky="ew")
        max_sat_slider.grid(row=1, column=3, sticky="ew")
        max_sat_entry.grid(row=1, column=4, sticky="e")

        min_val_entry.grid(row=2, column=1, sticky="w")
        min_val_slider.grid(row=2, column=2, sticky="ew")
        max_val_slider.grid(row=2, column=3, sticky="ew")
        max_val_entry.grid(row=2, column=4, sticky="e")

        setattr(self, f"{color_code}_min_hue_slider", min_hue_slider)
        setattr(self, f"{color_code}_max_hue_slider", max_hue_slider)
        setattr(self, f"{color_code}_min_sat_slider", min_sat_slider)
        setattr(self, f"{color_code}_max_sat_slider", max_sat_slider)
        setattr(self, f"{color_code}_min_val_slider", min_val_slider)
        setattr(self, f"{color_code}_max_val_slider", max_val_slider)

        setattr(self, f"{color_code}_min_hue_entry", min_hue_entry)
        setattr(self, f"{color_code}_max_hue_entry", max_hue_entry)
        setattr(self, f"{color_code}_min_sat_entry", min_sat_entry)
        setattr(self, f"{color_code}_max_sat_entry", max_sat_entry)
        setattr(self, f"{color_code}_min_val_entry", min_val_entry)
        setattr(self, f"{color_code}_max_val_entry", max_val_entry)

    def create_slider_with_entry(self, parent, label, initial_value, command):
        slider = tk.Scale(parent, from_=0, to=255, orient=tk.HORIZONTAL, command=command, length=330)
        slider.set(initial_value)
        entry = ttk.Entry(parent, width=5, font=("Helvetica", 10))
        entry.insert(0, str(initial_value))
        entry.bind("<Return>", lambda event, s=slider, e=entry: self.update_slider_from_entry(s, e))
        slider.config(command=lambda value, e=entry: self.update_entry_from_slider(value, e))
        return slider, entry

    def configure_grid(self):
        for i in range(9):
            self.root.grid_rowconfigure(i, weight=1, minsize=30)
        for i in range(5):
            self.root.grid_columnconfigure(i, weight=1, minsize=100)

    def update_slider_from_entry(self, slider, entry):
        try:
            value = int(entry.get())
        except ValueError:
            value = slider.get()
        slider.set(value)
        self.update_image()

    def update_entry_from_slider(self, value, entry):
        entry.delete(0, tk.END)
        entry.insert(0, str(value))
        self.update_image()

    def update_image(self, event=None):
        self.image_file = f"{self.directory}/color/{self.frame_idx:04d}.png"
        self.label_image.config(text=f"Image: {self.frame_idx:04d}.png")
        image = cv2.imread(self.image_file)
        if image is None:
            return

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        self.update_slider_values("r")
        self.update_slider_values("g")
        self.update_slider_values("b")

        mask_r = cv2.inRange(hsv_image, self.lower_bound(self.min_hue_r, self.min_sat_r, self.min_val_r), self.upper_bound(self.max_hue_r, self.max_sat_r, self.max_val_r))
        mask_g = cv2.inRange(hsv_image, self.lower_bound(self.min_hue_g, self.min_sat_g, self.min_val_g), self.upper_bound(self.max_hue_g, self.max_sat_g, self.max_val_g))
        mask_b = cv2.inRange(hsv_image, self.lower_bound(self.min_hue_b, self.min_sat_b, self.min_val_b), self.upper_bound(self.max_hue_b, self.max_sat_b, self.max_val_b))

        result_image = np.zeros_like(image)
        result_image[mask_r > 0] = [0, 0, 255]  # Red
        result_image[mask_g > 0] = [0, 255, 0]  # Green
        result_image[mask_b > 0] = [255, 0, 0]  # Blue

        self.display_image(self.canvas_orig, image)
        self.display_image(self.canvas_filtered, result_image)

    def lower_bound(self, min_hue, min_sat, min_val):
        return np.array([min_hue, min_sat, min_val])

    def upper_bound(self, max_hue, max_sat, max_val):
        return np.array([max_hue, max_sat, max_val])

    def update_slider_values(self, color_code):
        setattr(self, f"min_hue_{color_code}", getattr(self, f"{color_code}_min_hue_slider").get())
        setattr(self, f"max_hue_{color_code}", getattr(self, f"{color_code}_max_hue_slider").get())
        setattr(self, f"min_sat_{color_code}", getattr(self, f"{color_code}_min_sat_slider").get())
        setattr(self, f"max_sat_{color_code}", getattr(self, f"{color_code}_max_sat_slider").get())
        setattr(self, f"min_val_{color_code}", getattr(self, f"{color_code}_min_val_slider").get())
        setattr(self, f"max_val_{color_code}", getattr(self, f"{color_code}_max_val_slider").get())

    def display_image(self, canvas, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (400, 300))
        image = Image.fromarray(image)
        image_tk = ImageTk.PhotoImage(image=image)
        canvas.image_tk = image_tk
        canvas.create_image(0, 0, anchor=tk.NW, image=image_tk)

    def prev_frame(self):
        if self.frame_idx > 0:
            self.frame_idx -= 1
            self.update_image()

    def next_frame(self):
        self.frame_idx += 1
        self.update_image()

    def save_hsv_values(self):
        hsv_values = {
            "hsv_ranges": {
                "red": [
                    [self.min_hue_r, self.min_sat_r, self.min_val_r],
                    [self.max_hue_r, self.max_sat_r, self.max_val_r]
                ],
                "green": [
                    [self.min_hue_g, self.min_sat_g, self.min_val_g],
                    [self.max_hue_g, self.max_sat_g, self.max_val_g]
                ],
                "blue": [
                    [self.min_hue_b, self.min_sat_b, self.min_val_b],
                    [self.max_hue_b, self.max_sat_b, self.max_val_b]
                ]
            }
        }

        # write to file
        cfg_path = os.path.join(self.package_path,'configs/data_cfg.json')
        with open(cfg_path) as f:
            data_cfg = json.load(f)
        data_cfg["hsv_ranges"] = hsv_values.get("hsv_ranges")
        with open(cfg_path,'w') as f:
            json.dump(data_cfg, f, indent=4)
        print("HSV filter saved to ",cfg_path)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python hsv_filter_tuner.py <trialname>")
        sys.exit(1)

    directory = sys.argv[1]
    root = tk.Tk()
    app = HSVFilterApp(root, directory)
    root.mainloop()