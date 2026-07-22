import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. CONFIGURATION ---
script_dir = os.path.dirname(os.path.abspath(__file__))
# Remplace par le nom exact de ton CSV si besoin
CSV_FILE = os.path.join(script_dir, "../data/joint_positions_flat.csv")

# Architecture asymétrique de ContinuO
LEGS = {
    "Avant Gauche (FL)": ["FL_HAA", "FL_HFE", "FL_KFE"],
    "Avant Droite (FR)": ["FR_HAA", "FR_HFE", "FR_KFE"],
    "Arrière Gauche (HL)": ["HL_HAA", "HL_HFE", "HL_KFE", "HL_AFE"],
    "Arrière Droite (HR)": ["HR_HAA", "HR_HFE", "HR_KFE", "HR_AFE"],
}

# Longueurs arbitraires des segments (en mètres) pour la vue 2D
L_THIGH = 0.25
L_CALF = 0.25
L_FOOT = 0.15


# --- 2. LECTURE DU FICHIER CSV ---
def parse_csv(filepath):
    if not os.path.exists(filepath):
        print(f"Erreur : Le fichier '{filepath}' est introuvable.")
        exit(1)

    print(f"Chargement du fichier : {filepath}")
    df = pd.read_csv(filepath)

    # Identification automatique des colonnes "target_"
    target_cols = [c for c in df.columns if c.startswith("target_")]
    if not target_cols:
        print("Erreur : Aucune colonne commençant par 'target_' trouvée dans le CSV.")
        exit(1)

    frames = []
    # Transformation du DataFrame en une liste de dictionnaires
    for _, row in df.iterrows():
        state = {}
        for col in target_cols:
            joint_name = col.replace("target_", "")
            state[joint_name] = float(row[col])
        frames.append(state)

    return frames


frames = parse_csv(CSV_FILE)
print(f"Démarrage du visualiseur avec {len(frames)} étapes.")

# --- 3. AFFICHAGE ET ANIMATION 2D ---
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle("ContinuO - Cinématique de profil depuis CSV", fontsize=16)

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


# Interval=20 correspond à 50Hz (1000ms / 50 = 20)
ani = FuncAnimation(
    fig, update, frames=len(frames), interval=20, blit=False, repeat=True
)
plt.show()
