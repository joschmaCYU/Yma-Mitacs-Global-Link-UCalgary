#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tkinter as tk
from sensor_msgs.msg import JointState
import math

class CommandsVisuNode:

    def __init__(self):
        rospy.init_node("commands_visu_node")

        self.topic_name_onnx = "/joint_targets_rl"
        self.topic_name_csv = "/joint_targets_csv"

        self.joint_values = {}      # Pour l'ONNX
        self.joint_values_csv = {}  # Pour le CSV

        self.root = tk.Tk()
        self.root.title("Joint Target Viewer (ONNX vs CSV Replay)")

        self.labels = {}

        self.canvas_width = 700
        self.canvas_height = 500
        self.body_center_x = 350
        self.body_center_y = 250
        self.link1H, self.link2H, self.link3H = 80, 100, 50
        self.link1F, self.link2F = 100, 70

        # Double Subscriber
        rospy.Subscriber(self.topic_name_onnx, JointState, self.joint_callback, queue_size=1)
        rospy.Subscriber(self.topic_name_csv, JointState, self.csv_callback, queue_size=1)

        self.build_interface()
        self.update_gui()

    def build_interface(self):
        title = tk.Label(self.root, text="Received Joints Commands\n(ONNX values)", font=("Arial", 16, "bold"))
        title.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        for col, text in enumerate(["Joint", "Target position [rad]"]):
            label = tk.Label(self.root, text=text, font=("Arial", 12, "bold"), borderwidth=1, relief="solid", width=20)
            label.grid(row=1, column=col, padx=2, pady=2)

        self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.grid(row=0, column=3, rowspan=30, padx=20, pady=10)

        self.canvas.create_text(600, 20, text="Légende:", font=("Arial", 12, "bold"))
        self.canvas.create_text(600, 40, text="Robot ONNX (Couleurs)", fill="black")
        self.canvas.create_text(600, 60, text="Robot CSV (Gris)", fill="gray")

    def joint_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_values[name] = position

    def csv_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_values_csv[name] = position

    def update_gui(self):
        row = 2
        for joint_name in sorted(self.joint_values.keys()):
            if joint_name not in self.labels:
                name_label = tk.Label(self.root, text=joint_name, font=("Arial", 11), borderwidth=1, relief="solid", width=20)
                value_label = tk.Label(self.root, text="0.000", font=("Arial", 11), borderwidth=1, relief="solid", width=20)
                name_label.grid(row=row, column=0, padx=2, pady=2)
                value_label.grid(row=row, column=1, padx=2, pady=2)
                self.labels[joint_name] = value_label

            self.labels[joint_name].config(text=f"{self.joint_values[joint_name]: .4f}")
            row += 1

        self.draw_robot()
        self.root.after(50, self.update_gui)

    def run(self):
        self.root.mainloop()

    def draw_robot(self):
        self.canvas.delete("robots")
        cx, cy = self.body_center_x, self.body_center_y

        # Chassis
        self.canvas.create_rectangle(cx-100, cy-30, cx+100, cy+30, width=3, tags="robots")
        self.canvas.create_line(cx+100, cy+30, cx+200, cy-10, width=3, tags="robots")
        self.canvas.create_line(cx+200, cy-10, cx+200, cy-70, width=3, tags="robots")
        self.canvas.create_line(cx+100, cy-30, cx+200, cy-70, width=3, tags="robots")
        self.canvas.create_line(cx-0, cy-70, cx+200, cy-70, width=3, tags="robots")
        self.canvas.create_line(cx-100, cy-30, cx-0, cy-70, width=3, tags="robots")
        self.canvas.create_line(cx-100, cy+30, cx-0, cy-10, width=3, dash=1, tags="robots")
        self.canvas.create_line(cx-0, cy-10, cx-0, cy-70, width=3, dash=1, tags="robots")
        self.canvas.create_line(cx-0, cy-10, cx+200, cy-10, width=3, dash=1, tags="robots")

        Flegs = {"FL": (cx+200, cy-10, 1), "FR": (cx+100, cy+30, 1)}
        Hlegs = {"HL": (cx-0, cy-10, 1), "HR": (cx-100, cy+30, 1)}

        # Double dessin : Fantôme (CSV) puis Réel (ONNX)
        for draw_mode in ["csv", "onnx"]:
            if draw_mode == "csv":
                current_joints = self.joint_values_csv
                color_line, color_j = "lightgray", "lightgray"
            else:
                current_joints = self.joint_values
                color_line, color_j = "black", "red"

            for leg, (x0, y0, direction) in Hlegs.items():
                haa, hfe = current_joints.get(f"{leg}_HAA", 0.0), current_joints.get(f"{leg}_HFE", 0.0)
                kfe, afe = current_joints.get(f"{leg}_KFE", 0.0), current_joints.get(f"{leg}_AFE", 0.0)

                a1 = direction * math.pi / 2 + hfe
                x1, y1 = x0 + self.link1H * math.cos(a1), y0 + self.link1H * math.sin(a1)
                x2, y2 = x1 + self.link2H * math.cos(a1+kfe), y1 + self.link2H * math.sin(a1+kfe)
                x3, y3 = x2 + self.link3H * math.cos(a1+kfe+afe), y2 + self.link3H * math.sin(a1+kfe+afe)

                lat_x, lat_y = 50 * math.sin(haa), 20 * math.sin(haa)
                x0, x1, x2, x3 = x0+lat_x, x1+lat_x, x2+lat_x, x3+lat_x
                y0, y1, y2, y3 = y0-lat_y, y1-lat_y, y2-lat_y, y3-lat_y

                self.canvas.create_line(x0, y0, x1, y1, width=4, fill=color_line, tags="robots")
                self.canvas.create_line(x1, y1, x2, y2, width=4, fill=color_line, tags="robots")
                self.canvas.create_line(x2, y2, x3, y3, width=4, fill=color_line, tags="robots")

                self.canvas.create_oval(x0-5, y0-5, x0+5, y0+5, fill=color_j, tags="robots")
                self.canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill=color_j, tags="robots")
                self.canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill=color_j, tags="robots")
                self.canvas.create_oval(x3-5, y3-5, x3+5, y3+5, fill=color_j, tags="robots")

            for leg, (x0, y0, direction) in Flegs.items():
                haa, hfe = current_joints.get(f"{leg}_HAA", 0.0), current_joints.get(f"{leg}_HFE", 0.0)
                kfe = current_joints.get(f"{leg}_KFE", 0.0)

                a1 = direction * math.pi / 2 + hfe
                x1, y1 = x0 + self.link1F * math.cos(a1), y0 + self.link1F * math.sin(a1)
                x2, y2 = x1 + self.link2F * math.cos(a1+kfe), y1 + self.link2F * math.sin(a1+kfe)

                lat_x, lat_y = 50 * math.sin(haa), 20 * math.sin(haa)
                x0, x1, x2 = x0+lat_x, x1+lat_x, x2+lat_x
                y0, y1, y2 = y0-lat_y, y1-lat_y, y2-lat_y

                self.canvas.create_line(x0, y0, x1, y1, width=4, fill=color_line, tags="robots")
                self.canvas.create_line(x1, y1, x2, y2, width=4, fill=color_line, tags="robots")

                self.canvas.create_oval(x0-5, y0-5, x0+5, y0+5, fill=color_j, tags="robots")
                self.canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill=color_j, tags="robots")
                self.canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill=color_j, tags="robots")

if __name__ == "__main__":
    try:
        viewer = CommandsVisuNode()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
