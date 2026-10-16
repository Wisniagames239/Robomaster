#include <Wire.h>

// --- KLASA PDController (1:1 TWOJA LOGIKA) ---
class PDController {
  public:
    float kp, kd, out_min, out_max;
    float prev_error = 0;
    unsigned long last_time = 0;

    PDController(float p, float d, float mi, float ma) {
      kp = p; kd = d; out_min = mi; out_max = ma;
    }

    float update(float current_value) {
      unsigned long now = millis();
      float dt = (now - last_time) / 1000.0; 
      if (dt <= 0) return 0;

      float error = 0 - current_value; // Cel to 0
      float p_term = kp * error;
      float derivative = (error - prev_error) / dt;
      float d_term = kd * derivative;

      float output = p_term + d_term;
      
      if (output > out_max) output = out_max;
      if (output < out_min) output = out_min;

      prev_error = error;
      last_time = now;
      return output;
    }
};

// --- INICJALIZACJA (Parametry z Twojego Pythona) ---
PDController pd_track_yaw(350.0, 5.0, -200.0, 200.0);
// Regulator jazdy: wejściem będzie (dystans - 30), celem jest 0
PDController pd_drive(1.5, 0.1, -100.0, 100.0); 

#define LIDAR_ADDR 0x62
int current_distance = 0;
bool isConnected = false;
unsigned long lastGimbalExecTime = 0;
unsigned long lastDetectionTime = 0;
const int CMD_INTERVAL = 50; 

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  
  // Reset Lidara
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00); Wire.write(0x00);
  Wire.endTransmission();
  
  Serial.println("SYSTEM: START (STABILIZACJA + DOJAZD)");
}

void loop() {
  // 1. Nawiązywanie połączenia
  if (!isConnected) {
    if (millis() % 2000 < 20) {
      Serial1.print("command;");
    }
    if (Serial1.available()) {
      String r = Serial1.readStringUntil(';');
      if (r.indexOf("ok") != -1 || r.indexOf("Already") != -1) {
        isConnected = true;
        Serial1.print("robot mode free;"); delay(100);
        Serial1.print("AI push people on freq 10;"); delay(100);
        Serial.println("POŁĄCZONO");
      }
    }
    return;
  }

  // 2. Pobranie dystansu z Lidara
  updateLidar();

  // 3. Przetwarzanie danych AI
  if (Serial1.available()) {
    String response = Serial1.readStringUntil(';');
    if (response.indexOf("AI push people") != -1) {
      handleLogic(response);
      lastDetectionTime = millis();
    }
  }

  // 4. Zatrzymanie jeśli zgubi człowieka
  if (millis() - lastDetectionTime > 500) {
    Serial1.print("gimbal speed p 0 y 0;");
    Serial1.print("chassis speed x 0 y 0 z 0;");
  }
}

void handleLogic(String data) {
  unsigned long currentTime = millis();
  if (currentTime - lastGimbalExecTime < CMD_INTERVAL) return;

  int spaces = 0;
  for(int i=0; i<data.length(); i++) if(data[i] == ' ') spaces++;

  if (spaces > 3) {
    float x = getField(data, ' ', 4).toFloat();
    float y = getField(data, ' ', 5).toFloat();

    // --- STEROWANIE GIMBALEM (Logika 1:1 z Twojego kodu) ---
    // x to wartość 0-1, środek to 0.5. Błąd to 0.5 - x.
    float yaw_speed = pd_track_yaw.update(0.5 - x);
    float pitch_speed = (0.5 - y) * 180.0;

    // --- LOGIKA DOJAZDU (Tylko gdy gimbal "ustabilizuje wzrok") ---
    float drive_speed = 0;
    
    // Sprawdzamy czy człowiek jest blisko środka ekranu (stabilizacja)
    // abs(0.5 - x) < 0.1 oznacza błąd mniejszy niż 10% od środka
    if (abs(0.5 - x) < 0.1) {
      // Wyliczamy prędkość jazdy: (aktualny_dystans - cel)
      // Jeśli distance = 100, wejście = 70, PD zwróci wartość do jazdy w przód
      drive_speed = pd_drive.update(current_distance - 30);
      
      // Martwa strefa - nie drgaj jeśli jesteś +/- 3cm od celu
      if (abs(current_distance - 30) < 3) drive_speed = 0;
    } else {
      // Jeśli gimbal jeszcze goni człowieka, podwozie stoi
      drive_speed = 0;
    }

    // WYSYŁANIE KOMEND
    // 1. Gimbal
    Serial1.print("gimbal speed p "); Serial1.print((int)pitch_speed);
    Serial1.print(" y "); Serial1.print((int)yaw_speed); Serial1.print(";");

    // 2. Podwozie (chassis speed x to jazda przód/tył)
    // drive_speed musi mieć odpowiedni znak - w RoboMaster "+" to zazwyczaj przód
    Serial1.print("chassis speed x "); Serial1.print(drive_speed / 100.0); // Przelicz na m/s jeśli trzeba
    Serial1.print(" y 0 z 0;");

    lastGimbalExecTime = currentTime;
  }
}

// Odczyt Lidara
void updateLidar() {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00); Wire.write(0x04);
  Wire.endTransmission();
  
  delay(20); // Potrzebne dla stabilności Lidara

  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x8f);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(LIDAR_ADDR, 2);
    if (Wire.available() >= 2) {
      byte high = Wire.read();
      byte low = Wire.read();
      current_distance = (high << 8) | low;
    }
  }
}

String getField(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "0";
}