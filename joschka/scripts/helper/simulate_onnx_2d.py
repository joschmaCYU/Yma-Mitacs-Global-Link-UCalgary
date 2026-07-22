import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import onnxruntime as ort

# --- 1. CONFIGURATION ---
script_dir = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(script_dir, "../data/joint_positions_flat.csv")
ONNX_MODEL = os.path.join(
    script_dir, "../Policy/Exported_policies/flat_pushing_pt2.onnx"
)

ACTION_SCALE = 0.5

# === PARAMÈTRE LIDAR ===
# True = Essaie de lire les 187 points du lidar dans le CSV (obs_55 à obs_241)
# False = Force le lidar à 0 (terrain plat)
USE_CSV_LIDAR = False
# =========================

JOINT_ORDER = [
    "FL_HAA",
    "FR_HAA",
    "HL_HAA",
    "HR_HAA",
    "FL_HFE",
    "FR_HFE",
    "HL_HFE",
    "HR_HFE",
    "FL_KFE",
    "FR_KFE",
    "HL_KFE",
    "HR_KFE",
    "HL_AFE",
    "HR_AFE",
]

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
    "FL_HFE": 0.4102,
    "FR_HFE": 0.4102,
    "HL_HFE": -0.6981,
    "HR_HFE": -0.6981,
    "FL_KFE": -1.2716,
    "FR_KFE": -1.2716,
    "HL_KFE": 1.676,
    "HR_KFE": 1.676,
    "HL_AFE": -1.7219,
    "HR_AFE": -1.7219,
}

L_THIGH, L_CALF, L_FOOT = 0.25, 0.25, 0.15

# --- 2. CHARGEMENT ONNX ET CSV ---
if not os.path.exists(ONNX_MODEL):
    print(f"Erreur : Le modèle '{ONNX_MODEL}' est introuvable.")
    exit(1)

session = ort.InferenceSession(ONNX_MODEL)
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
num_obs = input_shape[1]

if not os.path.exists(CSV_FILE):
    print(f"Erreur : Le fichier '{CSV_FILE}' est introuvable.")
    exit(1)

df = pd.read_csv(CSV_FILE)
print(f"Génération des frames via ONNX pour {len(df)} étapes...")
if USE_CSV_LIDAR:
    print("Mode Lidar : ACTIVÉ (Tentative de lecture du CSV)")
else:
    print("Mode Lidar : DÉSACTIVÉ (Forcé à 0 - Terrain plat)")

frames = []

# --- 3. BOUCLE D'INFÉRENCE ---
for index, row in df.iterrows():
    # Création du vecteur d'observation (243 cases) rempli de zéros par défaut
    obs = np.zeros((1, num_obs), dtype=np.float32)

    # 1. Dynamique de base (Vitesses, Commandes, Positions, Actions) -> obs_0 à obs_54
    for i in range(55):
        if f"obs_{i}" in row:
            obs[0, i] = row[f"obs_{i}"]

    # 2. Gestion du Lidar (obs_55 à obs_241)
    if USE_CSV_LIDAR:
        try:
            if "obs_55" in row and "obs_241" in row:
                for i in range(55, 242):
                    obs[0, i] = row[f"obs_{i}"]
        except Exception as e:
            pass

    # 3. Gestion du time_remaining_s (Dernière observation)
    if num_obs >= 243:
        if "obs_242" in row:
            obs[0, 242] = row["obs_242"]
    elif "obs_55" in row and not USE_CSV_LIDAR:
        obs[0, 55] = row["obs_55"]

    # for i in range(len(obs)):
    #     print(obs[0, 0])
    #     print(obs[0, 1])

    # Inférence ONNX
    actions = session.run(None, {input_name: obs})[0][0]

    # Calcul des cibles absolues
    target_state = {}
    for i, joint in enumerate(JOINT_ORDER):
        absolute_target = base_state[joint] + (actions[i] * ACTION_SCALE)
        target_state[joint] = absolute_target

    frames.append(target_state)

print("Inférence terminée.")

print("Lancement de l'animation...")

# --- 4. AFFICHAGE ET ANIMATION 2D ---
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
title_status = "Lidar du CSV" if USE_CSV_LIDAR else "Lidar Forcé à 0"
fig.suptitle(f"ContinuO - Sortie ONNX ({title_status})", fontsize=16)

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
    ax.axhline(0, color="black", linewidth=2)
    ax.axvline(0, color="gray", linewidth=1, linestyle=":")
    ax.set_title(title)

    (line,) = ax.plot([], [], "o-", lw=5, markersize=8, color="forestgreen")
    lines_dict[title] = line

plt.tight_layout()


def update(frame_idx):
    state = frames[frame_idx]

    for title, joints in LEGS.items():
        hfe = state.get(joints[1], 0.0)
        kfe = state.get(joints[2], 0.0)

        x_coords, y_coords = [0.0], [0.0]

        x_knee = L_THIGH * np.sin(hfe)
        y_knee = -L_THIGH * np.cos(hfe)
        x_coords.append(x_knee)
        y_coords.append(y_knee)

        x_ankle = x_knee + L_CALF * np.sin(hfe + kfe)
        y_ankle = y_knee - L_CALF * np.cos(hfe + kfe)
        x_coords.append(x_ankle)
        y_coords.append(y_ankle)

        if len(joints) == 4:
            afe = state.get(joints[3], 0.0)
            x_foot = x_ankle + L_FOOT * np.sin(hfe + kfe + afe)
            y_foot = y_ankle - L_FOOT * np.cos(hfe + kfe + afe)
            x_coords.append(x_foot)
            y_coords.append(y_foot)

        lines_dict[title].set_data(x_coords, y_coords)

    return list(lines_dict.values())


ani = FuncAnimation(
    fig, update, frames=len(frames), interval=20, blit=False, repeat=True
)
plt.show()
