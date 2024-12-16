/*
 * Projet : Stabilisation d'un drone en hauteur (axe Z) avec un contrôleur PID - Sortie V seule
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

// Plage de sortie PWM (en microsecondes) pour V
const int pwmMin = 991;
const int pwmMax = 2016;

//======================= Variables globales =============================

// Variables PID
float lastError = 0.0f;
float integral = 0.0f;
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

//======================= Fonction CommandeV ==============================

/*
 * Fonction CommandeV :
 *  - Calcule la commande V (PWM) pour le contrôleur de vol basée sur une consigne et une altitude mesurée
 *  - Entrées :
 *      - currentSetpoint : consigne d'altitude (en cm)
 *      - currentAltitude : altitude mesurée (en cm)
 *  - Sortie :
 *      - Commande V (valeur PWM entre pwmMin et pwmMax)
 */
int CommandeV(float currentSetpoint, float currentAltitude) {
  // Calcul de l'erreur
  float error = currentSetpoint - currentAltitude;

  // Calcul PID
  float P = Kp * error;
  integral += error * dt;
  float I = Ki * integral;
  float derivative = (error - lastError) / dt;
  float D = Kd * derivative;

  float pidOutput = P + I + D;
  lastError = error;

  // Mapper directement la sortie PID vers la plage PWM
  int vCommand = (int)mapFloat(pidOutput, -Kp * 150.0f, Kp * 150.0f, pwmMin, pwmMax);
  vCommand = constrain(vCommand, pwmMin, pwmMax);

  return vCommand;
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

  // Initialisation de la commande V à un niveau neutre
  vServo.writeMicroseconds((pwmMin + pwmMax) / 2);
}

//======================= Boucle principale ===============================
void loop() {
  static float currentSetpoint = 0.0f;
  static float lastValidAltitude = 0.0f;

  // Augmenter progressivement la consigne jusqu'à une cible de 150 cm
  if (currentSetpoint < 150.0f) {
    currentSetpoint += 20.0f * dt; // Incrémenter la consigne (MODIFIER ICI LA RAMPE)
    currentSetpoint = constrainFloat(currentSetpoint, 0.0f, 150.0f);
  }

  // Lecture de l'altitude
  float distCM = measureDistanceCM();
  float currentAltitude;
  if (distCM < 0) {
    // Mesure invalide, on utilise la dernière valeur connue
    currentAltitude = lastValidAltitude;
  } else {
    currentAltitude = distCM;
    lastValidAltitude = currentAltitude;
  }

  // Calcul de la commande V
  int vCommand = CommandeV(currentSetpoint, currentAltitude);

  // Envoi de la commande V au contrôleur de vol
  vServo.writeMicroseconds(vCommand);

  // Affichage sur le moniteur série
  Serial.print("Consigne : ");
  Serial.print(currentSetpoint, 2);
  Serial.print(" cm | Altitude : ");
  Serial.print(currentAltitude, 2);
  Serial.print(" cm | PWM V : ");
  Serial.println(vCommand);

  // Pause pour respecter l'intervalle d'échantillonnage
  delay(100);
}
