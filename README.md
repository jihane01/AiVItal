# 🩺 AiVital Health Monitor

**Système AIoT de Monitoring Médical Prédictif**
*AiVital Track 2026*

AiVital Health Monitor est une solution complète de surveillance médicale connectée (IoT) qui collecte les signes vitaux d'un patient via un ESP32, les analyse grâce à un modèle d'Intelligence Artificielle (Hugging Face) et un LLM (Groq), puis déclenche des alertes médicales automatiques et met à jour un tableau de bord en temps réel.

---

## 📑 Table des Matières
1. [Vue d'ensemble](#-vue-densemble)
2. [Architecture du système](#-architecture-du-système)
3. [Composants matériels](#-composants-matériels)
4. [Stack logicielle](#-stack-logicielle)
5. [Installation & Configuration](#-installation--configuration)


---

## 🔍 Vue d'ensemble

Le monitoring médical traditionnel est souvent **réactif** : il alerte après que le seuil critique est franchi. **AiVital Health Monitor** change cette approche en utilisant l'**Intelligence Artificielle prédictive** pour détecter les anomalies avant qu'elles ne deviennent critiques.

Ce prototype (POC/MVP) démontre la faisabilité d'une chaîne complète :
> **Capteurs IoT** -> **MQTT** -> **Orchestration (Fusion AI)** -> **Modèle IA (Hugging Face)** -> **LLM (Groq)** -> **Alertes** & **Dashboard**

---

## 🏗️ Architecture du système

Le projet s'articule autour de 3 composantes principales :

1. **Couche Physique (IoT)** : L'ESP32 collecte les données vitale et environnementales.
2. **Couche Intelligence (Backend)** : Le workflow Fusion AI (AbaFusion) orchestre l'analyse IA et le routage.
3. **Couche Interface (UI/UX)** : Dashboard Web "Glassmorphism" pour la visualisation temps réel.

> *Pour plus de détails, consultez le diagramme BPMN dans le dossier `/docs`.*

---

## 🧩 Composants matériels

Le firmware est conçu pour fonctionner avec les capteurs suivants (connectés via bus I2C partagé) :

| Composant | Fonction | Interface |
| :--- | :--- | :--- |
| **ESP32 (MakerPoints)** | Microcontrôleur principal | - |
| **MAX30100** | Fréquence cardiaque (BPM) & SpO2 | I2C |
| **MLX90614** | Température corporelle (sans contact) | I2C |
| **MPU6050** | Podomètre & détection de chute | I2C |
| **BME280** | Température, humidité, pression ambiante | I2C |
| **LCD 16x2 (I2C)** | Affichage des écrans de navigation | I2C |
| **Clavier matriciel** | Saisie du profil & du mode | GPIO |
| **Buzzer & LEDs** | Alertes sonores et visuelles locales | GPIO |

**Brochage (Pinout) :**
*   **SDA** : GPIO 21
*   **SCL** : GPIO 22
*   **Bouton 1 (Menu)** : GPIO 35
*   **Bouton 2 (SOS)** : GPIO 34
*   **Buzzer** : GPIO 26
*   **LED Verte** : GPIO 33 | **LED Rouge** : GPIO 32

---

## 💻 Stack logicielle

*   **Firmware** : C++ / Arduino IDE (PlatformIO)
*   **Transport** : MQTT via HiveMQ Cloud (TLS/SSL Port 8883)
*   **Backend / Orchestration** : Fusion AI (AbaFusion)
*   **Modèle IA** : Hugging Face (API Gradio, PyTorch)
*   **LLM de raisonnement clinique** : Groq (API)
*   **Base de données** : PostgreSQL / Google Sheets
*   **Notifications** : Slack, Gmail
*   **Dashboard** : HTML5 / CSS3 / JavaScript (Chart.js, MQTT.js)

---

## ⚙️ Installation & Configuration

### 1. Prérequis
*   Arduino IDE (avec support ESP32) ou PlatformIO.
*   Compte HiveMQ Cloud (avec utilisateur MQTT).
*   Compte Hugging Face (avec Space actif pour le modèle).
*   Clé API Groq (et OpenRouter/Fusion AI).
*   Compte Google Cloud (pour Google Docs/Sheets/Gmail).

### 2. Configuration du Firmware
**IMPORTANT : Ne jamais versionner vos secrets.**
1. Créez un fichier `secrets.h` à la racine du firmware.
2. Ajoutez `secrets.h` à votre fichier `.gitignore`.
3. Remplissez `secrets.h` avec vos identifiants (voir exemple ci-dessous) :

```cpp
#ifndef SECRETS_H
#define SECRETS_H

const char* WIFI_SSID     = "Votre_Réseau_WiFi";
const char* WIFI_PASS     = "Votre_Mot_de_Passe_WiFi";

const char* MQTT_HOST     = "d20b650b186d488a9a7d95dfcb4c6954.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883;
const char* MQTT_USER     = "makerboard_user";
const char* MQTT_PASS     = "Votre_Mot_de_Passe_HiveMQ";

#endif
