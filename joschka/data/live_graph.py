import serial
import re
import matplotlib.pyplot as plt

# ==========================================
# CONFIGURATION
# ==========================================
PORT = "/dev/ttyACM0"  # Ton port Arduino (modifie si nécessaire)
BAUD = 115200  # Doit être identique au Serial.begin(...) de ton Arduino
MAX_POINTS = 50  # Nombre de points affichés simultanément (historique glissant)

# Structure pour stocker l'historique des données
data = {
    "MOT1": {"pos": [], "vit": [], "pwm": []},
    "MOT2": {"pos": [], "vit": [], "pwm": []},
    "MOT3": {"pos": [], "vit": [], "pwm": []},
}
derniere_cible = 0

# Expression régulière pour parser ton format de log spécifique
regex = r"(MOT\d)\s*\|\s*Cible:([\-\d]+)\s*\|\s*Pos:([\-\d]+)\s*\|\s*Vit(?:\(rpm\))?:([\-\d]+)\s*\|\s*PWM:([\-\d]+)"

# Couleurs personnalisées pour chaque moteur
colors = {"MOT1": "#4472C4", "MOT2": "#ED7D31", "MOT3": "#70AD47"}

# Configuration de la fenêtre Matplotlib (3 sous-graphiques empilés)
plt.ion()  # Activation du mode interactif pour le temps réel
fig, (ax_pos, ax_vit, ax_pwm) = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle("Suivi Temporel des Moteurs", fontsize=14, fontweight="bold")

print(f"Tentative de connexion sur {PORT} à {BAUD} bauds...")
print("IMPORTANT : Ferme le moniteur série de l'Arduino IDE !")

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print("Connexion réussie. Écoute du flux en cours... (Ctrl+C pour quitter)")

    while True:
        # Lire une ligne textuelle venant de l'Arduino
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        # Analyser la ligne avec la regex
        match = re.search(regex, line)
        if match:
            mot = match.group(1)
            cible = int(match.group(2))
            pos = int(match.group(3))
            vit = int(match.group(4))
            pwm = int(match.group(5))

            derniere_cible = cible  # Mise à jour de la consigne

            # Ajouter les nouvelles valeurs dans l'historique du moteur concerné
            data[mot]["pos"].append(pos)
            data[mot]["vit"].append(vit)
            data[mot]["pwm"].append(pwm)

            # Maintenir la taille de la fenêtre glissante
            if len(data[mot]["pos"]) > MAX_POINTS:
                data[mot]["pos"].pop(0)
                data[mot]["vit"].pop(0)
                data[mot]["pwm"].pop(0)

            # --- REDESSINER LES GRAPHIQUES ---
            ax_pos.cla()
            ax_vit.cla()
            ax_pwm.cla()

            # 1. Graphique des Positions
            ax_pos.set_title("Positions vs Cible")
            ax_pos.set_ylabel("Ticks / encodeur")
            for m in ["MOT1", "MOT2", "MOT3"]:
                if data[m]["pos"]:
                    ax_pos.plot(data[m]["pos"], label=m, color=colors[m], linewidth=1.5)
            ax_pos.axhline(
                y=derniere_cible,
                color="red",
                linestyle="--",
                label="Cible",
                linewidth=1.2,
            )
            ax_pos.legend(loc="upper left")
            ax_pos.grid(True, linestyle=":", alpha=0.6)

            # 2. Graphique des Vitesses
            ax_vit.set_title("Vitesses")
            ax_vit.set_ylabel("RPM")
            for m in ["MOT1", "MOT2", "MOT3"]:
                if data[m]["vit"]:
                    ax_vit.plot(data[m]["vit"], label=m, color=colors[m], linewidth=1.5)
            ax_vit.legend(loc="upper left")
            ax_vit.grid(True, linestyle=":", alpha=0.6)

            # 3. Graphique des signaux PWM
            ax_pwm.set_title("Signaux de Commande (PWM)")
            ax_pwm.set_ylabel("Valeur PWM")
            ax_pwm.set_xlabel("Échantillons récents")
            for m in ["MOT1", "MOT2", "MOT3"]:
                if data[m]["pwm"]:
                    ax_pwm.plot(data[m]["pwm"], label=m, color=colors[m], linewidth=1.5)
            ax_pwm.legend(loc="upper left")
            ax_pwm.grid(True, linestyle=":", alpha=0.6)

            # Rafraîchissement de la fenêtre
            plt.tight_layout()
            plt.pause(
                0.001
            )  # Pause minimale indispensable pour laisser l'IHM se dessiner

except serial.SerialException as e:
    print(f"\nErreur port série : {e}")
    print("Vérifie que l'Arduino est branché et que le moniteur de l'IDE est fermé.")
except KeyboardInterrupt:
    print("\nScript interrompu par l'utilisateur.")
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
        print("Port série déconnecté proprement.")
