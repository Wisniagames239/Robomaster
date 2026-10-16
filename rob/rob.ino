// Arduino Mega - RoboMaster S1: JAZDA NA PODSTAWIE KODU PYTHON

bool isConnected = false;
bool isInitialized = false; // Nowa flaga inicjalizacji
unsigned long lastAttemptTime = 0;
const int retryInterval = 1000;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("--- START ---");
  delay(1000);
}

void loop() {
  unsigned long startTime; 

  // 1. Łączenie (wykonuje się tylko raz na początku)
  if (!isConnected) {
    if (millis() - lastAttemptTime >= retryInterval) {
      lastAttemptTime = millis();
      Serial1.print("command;"); 
    }
    if (Serial1.available()) {
      String r = Serial1.readStringUntil(';');
      if (r.indexOf("ok") != -1 || r.indexOf("Already") != -1) {
        isConnected = true;
        Serial.println(">>> POŁĄCZONO <<<");
      }
    }
    return;
  }

  // 2. Inicjalizacja (wykonuje się TYLKO RAZ po połączeniu)
  if (!isInitialized) {
    Serial.println("Inicjalizacja robota...");
    sendCmd("robot mode free;"); 
    sendCmd("gimbal recenter;");
    delay(1000); // Czekamy na powrót gimbala
    isInitialized = true;
    Serial.println("Gotowy do jazdy.");
  }

  // 3. PĘTLA JAZDY (to będzie się powtarzać w kółko)
  Serial.println(">>> JAZDA DO PRZODU <<<");
  
  // Zwiększyłem prędkość do 200 dla lepszej stabilności
  startTime = millis();
  while(millis() - startTime < 1000) {
    // Wysłanie komendy raz i krótki delay
    Serial1.print("chassis wheel w1 150 w2 -150 w3 150 w4 -150;"); 
    delay(50); 
    while(Serial1.available()) Serial1.read(); // Czyścimy odpowiedzi
  }

  // Krótki STOP dla stabilności przed obrotem
  Serial1.print("chassis wheel w1 0 w2 0 w3 0 w4 0;");
  delay(200);

  Serial.println(">>> OBRÓT <<<");
  sendCmd("robot mode chassis_lead;"); 
  
  startTime = millis();
  while(millis() - startTime < 1500) {
    Serial1.print("chassis speed x 0 y 0 z 60;"); 
    delay(100);
    while(Serial1.available()) Serial1.read();
  }
  
  // STOP przed końcem pętli
  Serial1.print("chassis speed x 0 y 0 z 0;");
  sendCmd("chassis stop;");
  sendCmd("blaster fire;");
  
  delay(500); // Krótka pauza przed kolejnym cyklem, żeby robot nie "wariował"
}

void sendCmd(String cmd) {
  while(Serial1.available()) Serial1.read();
  Serial1.print(cmd);
  delay(150); // Trochę dłuższy czas na przetworzenie komendy trybu
  while(Serial1.available()) Serial1.read();
}