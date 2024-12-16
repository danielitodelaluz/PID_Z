/*
 * Projet : Stabilisation d'un drone en hauteur (axe Z) avec un contrôleur PID - Sortie V seule
 * Matériel : 
 *   - Microcontrôleur : Raspberry Pi Pico (PlatformIO + Arduino framework)
 *   - Capteur ultrason : HC-SR04 (trigPin = 2, echoPin = 3)
 *   - Une sortie PWM vers le contrôleur de vol pour la commande V (GPIO 5)
 *
 * Fonctionnalités :
 *   - Lecture de l'altitude via HC-SR04 (en cm, puis conversion en m)
 *   - Contrôleur PID pour maintenir l'altitude à une consigne dynamique
 *   - Mise à jour du PID toutes les 100 ms
 *   - Génération d'un seul signal PWM (V) vers le contrôleur de vol
 *   - Affichage sur le moniteur série des valeurs importantes (consigne, altitude, erreur, P, I, D, sortie PID)
 *   - Gestion des mesures invalides du capteur (utiliser la dernière valeur valide)
 */

#include <Arduino.h>
#include <Servo.h>

//======================= Constantes de configuration ======================

// Broches du capteur ultrason
const uint8_t trigPin = 2;
const uint8_t echoPin = 3;

// Broche PWM pour la commande verticale (V)
const uint8_t vPin = 5; 

// PID constants (ajustables)
float Kp = 30.0f;
float Ki = 0.0f;
float Kd = 10.0f;

// Temps d'échantillonnage (en secondes)
const float dt = 0.1f; // 100 ms

// Consigne cible (en mètres)
const float targetSetpoint = 1.5f;

// Taux d'augmentation de la consigne (m/s)
const float setpointIncrement = 0.1f;

// Plage de sortie PWM (en microsecondes) pour V
const int pwmMin = 991;
const int pwmMax = 2016;

//======================= Variables globales =============================

// Variables pour l'altitude
float currentAltitude = 0.0f; 
float lastValidAltitude = 0.0f; 

// Consigne actuelle (commence à 0)
float currentSetpoint = 0.0f;

// Variables PID
float error = 0.0f;
float lastError = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float pidOutput = 0.0f;

// Chronométrage
unsigned long lastUpdateTime = 0; 

// Servo pour la commande V
Servo vServo;

//======================= Fonctions utilitaires ===========================

// Mesure de la distance avec le HC-SR04 en cm
float measureDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout 30 ms
  
  if (duration == 0) {
    return -1.0f; // Aucune mesure valide
  }
  
  float distanceCM = (duration * 0.0343f) / 2.0f;
  return distanceCM;
}

// Convertir cm en m
float cmToM(float cm) {
  return cm / 100.0f;
}

// Limiter une valeur entre un min et un max
float constrainFloat(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

// Mapper une valeur d'une plage à une autre
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//======================= Setup ===========================================
void setup() {
  // Initialisation du moniteur série
  Serial.begin(115200);
  delay(1000);
  Serial.println("Demarrage du systeme...");

  // Configuration des broches du capteur ultrason
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attache du servo pour la commande V
  vServo.attach(vPin);

  // Initialisation de la commande V à un niveau neutre (par ex. milieu)
  vServo.writeMicroseconds((pwmMin + pwmMax) / 2);

  // Initialisation du dernier temps de mise à jour PID
  lastUpdateTime = millis();
}

//======================= Boucle principale ===============================
void loop() {
  unsigned long currentTime = millis();
  
  // Mise à jour du PID toutes les 100 ms
  if (currentTime - lastUpdateTime >= 100) {
    lastUpdateTime = currentTime;

    // Augmenter progressivement la consigne
    if (currentSetpoint < targetSetpoint) {
      currentSetpoint += setpointIncrement * dt; // Incrémente la consigne
      currentSetpoint = constrainFloat(currentSetpoint, 0.0f, targetSetpoint); // Limite à targetSetpoint
    }

    // Lecture de l'altitude
    float distCM = measureDistanceCM();
    if (distCM < 0) {
      // Mesure invalide, on utilise la dernière valeur connue
      currentAltitude = lastValidAltitude;
    } else {
      currentAltitude = cmToM(distCM);
      lastValidAltitude = currentAltitude;
    }

    // Calcul de l'erreur
    error = currentSetpoint - currentAltitude;

    // Calcul PID
    float P = Kp * error;
    integral += error * dt;
    float I = Ki * integral;
    derivative = (error - lastError) / dt;
    float D = Kd * derivative;

    pidOutput = P + I + D;
    lastError = error;

    // Conversion de la sortie PID en commande V (PWM)
    // On limite pidOutput pour éviter des valeurs trop extrêmes
    float pidConstrained = constrainFloat(pidOutput, -2.0f * Kp, 2.0f * Kp);

    // Mapping de la plage PID vers la plage PWM
    int vCommand = (int)mapFloat(pidConstrained, -Kp*2.0f, Kp*2.0f, pwmMin, pwmMax);
    vCommand = constrain(vCommand, pwmMin, pwmMax);

    // Envoi de la commande V au contrôleur de vol
    vServo.writeMicroseconds(vCommand);

    // Affichage sur le moniteur série
    Serial.print("Consigne : ");
    Serial.print(currentSetpoint, 2);
    Serial.print(" m | Altitude : ");
    Serial.print(currentAltitude, 2);
    Serial.print(" m | Erreur : ");
    Serial.print(error, 2);
    Serial.println(" m");

    Serial.print("P : ");
    Serial.print(P, 2);
    Serial.print(" | I : ");
    Serial.print(I, 2);
    Serial.print(" | D : ");
    Serial.print(D, 2);
    Serial.print(" | Sortie PID (V) : ");
    Serial.print(pidOutput, 2);
    Serial.print(" -> PWM V : ");
    Serial.print(vCommand);
    Serial.println(" µs\n");
  }

  // Autres tâches si nécessaire
}
