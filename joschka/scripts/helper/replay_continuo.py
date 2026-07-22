import re
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. CONFIGURATION ---
script_dir = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = os.path.join(script_dir, "robot_log.txt")

LEGS = {
    "Avant Gauche (FL)": ["FL_HAA", "FL_HFE", "FL_KFE"],
    "Avant Droite (FR)": ["FR_HAA", "FR_HFE", "FR_KFE"],
    "Arrière Gauche (HL)": ["HL_HAA", "HL_HFE", "HL_KFE", "HL_AFE"],
    "Arrière Droite (HR)": ["HR_HAA", "HR_HFE", "HR_KFE", "HR_AFE"],
}

base_state = {
    "FL_HAA": 0.0,
    "FR_HAA": 0.0,
    "HL_HAA": 0.0,
    "HR_HAA": 0.0,
    "FL_HFE": 0.41,
    "FR_HFE": 0.41,
    "HL_HFE": -0.70,
    "HR_HFE": -0.70,
    "FL_KFE": -1.27,
    "FR_KFE": -1.27,
    "HL_KFE": 1.68,
    "HR_KFE": 1.68,
    "HL_AFE": -1.72,
    "HR_AFE": -1.72,
}

# Longueurs arbitraires des segments (en mètres) pour la vue 2D
L_THIGH = 0.25
L_CALF = 0.25
L_FOOT = 0.15


# --- 2. PARSER LE FICHIER ---
def parse_logs(filepath):
    frames = []
    current_state = base_state.copy()
    current_time = -1.0
    pattern = re.compile(r"\[.*?,\s*(\d+\.\d+)\]:\s*([A-Z_]+):\s*([-\d\.]+)")

    if not os.path.exists(filepath):
        print(f"Erreur : Le fichier '{filepath}' est introuvable.")
        exit(1)

    with open(filepath, "r") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                t = float(match.group(1))
                joint = match.group(2)
                val = float(match.group(3))

                if t != current_time and current_time != -1.0:
                    frames.append(current_state.copy())

                current_time = t
                current_state[joint] = val

    if current_state:
        frames.append(current_state.copy())
    return frames


frames = parse_logs(LOG_FILE)
if not frames:
    print("Aucune donnée valide trouvée.")
    exit(1)

# --- 3. AFFICHAGE ET ANIMATION 2D ---
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle("ContinuO - Cinématique de profil (Vue Latérale)", fontsize=16)

ax_map = {
    "Avant Gauche (FL)": axs[0, 0],
    "Avant Droite (FR)": axs[0, 1],
    "Arrière Gauche (HL)": axs[1, 0],
    "Arrière Droite (HR)": axs[1, 1],
}

lines_dict = {}

for title, ax in ax_map.items():
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylim(-0.8, 0.2)
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.axhline(0, color="black", linewidth=2)  # Représente le châssis
    ax.axvline(0, color="gray", linewidth=1, linestyle=":")
    ax.set_title(title)

    # Initialisation de la ligne articulée
    (line,) = ax.plot([], [], "o-", lw=5, markersize=8, color="steelblue")
    lines_dict[title] = line

plt.tight_layout()


def update(frame_idx):
    state = frames[frame_idx]

    for title, joints in LEGS.items():
        # joints[1] = HFE, joints[2] = KFE, joints[3] = AFE (si présent)
        hfe = state.get(joints[1], 0.0)
        kfe = state.get(joints[2], 0.0)

        # Calcul de la cinématique (Hanche = Origine)
        x_coords = [0.0]
        y_coords = [0.0]

        # Genou
        x_knee = L_THIGH * np.sin(hfe)
        y_knee = -L_THIGH * np.cos(hfe)
        x_coords.append(x_knee)
        y_coords.append(y_knee)

        # Cheville
        x_ankle = x_knee + L_CALF * np.sin(hfe + kfe)
        y_ankle = y_knee - L_CALF * np.cos(hfe + kfe)
        x_coords.append(x_ankle)
        y_coords.append(y_ankle)

        # Pied (uniquement pour les pattes arrière qui ont 4 DOF)
        if len(joints) == 4:
            afe = state.get(joints[3], 0.0)
            x_foot = x_ankle + L_FOOT * np.sin(hfe + kfe + afe)
            y_foot = y_ankle - L_FOOT * np.cos(hfe + kfe + afe)
            x_coords.append(x_foot)
            y_coords.append(y_foot)

        lines_dict[title].set_data(x_coords, y_coords)

    return list(lines_dict.values())


ani = FuncAnimation(
    fig, update, frames=len(frames), interval=50, blit=False, repeat=True
)
plt.show()
