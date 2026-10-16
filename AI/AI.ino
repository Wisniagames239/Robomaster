// Arduino Mega - RoboMaster S1: PORT LOGIKI Z ROBOMASTERV2.PY
// Implementacja 1:1 algorytmu PD oraz sterowania gimbalem

// --- KLASA PDController (Kopia logiki z Twojego Pythona) ---
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
      float dt = (now - last_time) / 1000.0; // Różnica czasu w sekundach
      if (dt <= 0) return 0;

      float error = 0 - current_value; // Cel to 0 (środek błędu)
      float p_term = kp * error;
      float derivative = (error - prev_error) / dt;
      float d_term = kd * derivative;

      float output = p_term + d_term;
      
      // Limitowanie wyjścia (output_limits)
      if (output > out_max) output = out_max;
      if (output < out_min) output = out_min;

      prev_error = error;
      last_time = now;
      return output;
    }
};

// --- Inicjalizacja obiektów i zmiennych ---
// Parametry identyczne jak w Twoim Pythonie: kp=350, kd=5, limits=(-200, 200)
PDController pd_track_yaw(350.0, 5.0, -200.0, 200.0);

bool isConnected = false;
bool aiActive = true; 
unsigned long lastAttemptTime = 0;
unsigned long lastGimbalExecTime = 0;
const int CMD_INTERVAL = 50; // 0.05s (zgodnie z CMD_INTERVAL w Pythonie)

void setup() {
  Serial.begin(115200);   // Monitor PC
  Serial1.begin(115200);  // RoboMaster S1
  Serial.println("--- SYSTEM START: LOGIKA PYTHON 1:1 ---");
  Serial.println("Komendy: 'on' / 'off' (sterowanie AI)");
}

void loop() {
  // 1. Nawiązywanie połączenia (Twoja sprawdzona logika)
  if (!isConnected) {
    if (millis() - lastAttemptTime >= 2000) {
      lastAttemptTime = millis();
      Serial.println("Próba połączenia: wysyłam 'command;'...");
      while(Serial1.available()) Serial1.read();
      Serial1.print("command;");
    }
  }

  // 2. Obsługa komend ON/OFF z Serial Monitora
  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "off") {
      aiActive = false;
      Serial1.print("AI push people off;");
      delay(50);
      Serial1.print("gimbal speed p 0 y 0;");
      Serial.println(">>> TRYB AI WYŁĄCZONY <<<");
    } else if (cmd == "on") {
      aiActive = true;
      Serial1.print("AI push people on freq 10;");
      Serial.println(">>> TRYB AI WŁĄCZONY <<<");
    }
  }

  // 3. Odbieranie danych i Inicjalizacja
  if (Serial1.available()) {
    String response = Serial1.readStringUntil(';');
    
    // Potwierdzenie połączenia i WYCISZENIE
    if (response.indexOf("ok") != -1 || response.indexOf("Already") != -1) {
      if (!isConnected) {
        isConnected = true;
        Serial.println(">>> POŁĄCZONO! <<<");

        // --- WYCISZENIE ROBOTA ---
        Serial1.print("audio off;");            // Wyłączenie pikania SDK
        delay(100);
        Serial1.print("robot audio volume 0;"); // Głośność systemowa na 0
        delay(100);

        // Inicjalizacja (identyczna kolejność jak w Pythonie)
        Serial1.print("stream on;"); delay(150);
        Serial1.print("robot mode free;"); delay(150);
        Serial1.print("AI push people on freq 10;"); delay(150);
        Serial.println(">>> ROBOT GOTOWY I WYCISZONY <<<");
      }
    }

    // Przetwarzanie danych AI (Port funkcji on_detect_person)
    if (aiActive && response.indexOf("AI push people") != -1) {
      handlePythonLogic(response);
    }
  }
}

// --- Odwzorowanie funkcji on_detect_person z robomasterv2.py ---
void handlePythonLogic(String data) {
  unsigned long currentTime = millis();
  
  // Throttling (Python: if current_time - last_gimbal_exec_time < CMD_INTERVAL: return)
  if (currentTime - lastGimbalExecTime < CMD_INTERVAL) return;

  // Analiza czy linia zawiera dane o pozycji (czy wykryto osobę)
  int spaces = 0;
  for(int i=0; i<data.length(); i++) if(data[i] == ' ') spaces++;

  if (spaces > 3) { 
    // person_info = [selected_person]
    float x = getField(data, ' ', 4).toFloat();
    float y = getField(data, ' ', 5).toFloat();

    // LOGIKA STEROWANIA Z PYTHONA:
    // yaw_speed = pd_track_yaw.update(0.5 - x)
    // pitch_speed = (0.5 - y) * 180
    
    // UWAGA: Jeśli robot "ucieka", zmień (0.5 - x) na (x - 0.5)
    float yaw_speed = pd_track_yaw.update(0.5 - x);
    float pitch_speed = (0.5 - y) * 180.0;

    // ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
    Serial1.print("gimbal speed p ");
    Serial1.print((int)pitch_speed);
    Serial1.print(" y ");
    Serial1.print((int)yaw_speed);
    Serial1.print(";");

    lastGimbalExecTime = currentTime;
  } else {
    // Jeśli nikt nie jest wykryty: ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
    Serial1.print("gimbal speed p 0 y 0;");
    lastGimbalExecTime = currentTime;
  }
}

// Funkcja pomocnicza do wycinania danych z ciągu tekstowego
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
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}