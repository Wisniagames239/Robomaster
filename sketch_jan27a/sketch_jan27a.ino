// Arduino Mega - RoboMaster S1 Auto-Connect
// TX1 (Pin 18) -> RX RoboMaster (przez konwerter napięć!)
// RX1 (Pin 19) -> TX RoboMaster

bool isConnected = false;
unsigned long lastAttemptTime = 0;
const int retryInterval = 2000; // Próbuj co 2 sekundy

void setup() {
  Serial.begin(115200);  // Konsola PC
  Serial1.begin(115200); // RoboMaster S1

  Serial.println("--- Start Systemu ---");
  Serial.println("Czekam na gotowość RoboMastera S1...");
  
  // Dajemy mu chwilę na start, jeśli włączamy zasilanie jednocześnie
  delay(1000); 
}

void loop() {
  // 1. Procedura nawiązywania połączenia
  if (!isConnected) {
    unsigned long currentMillis = millis();
    
    // Wysyłaj "command;" co 2 sekundy, dopóki nie połączysz
    if (currentMillis - lastAttemptTime >= retryInterval) {
      lastAttemptTime = currentMillis;
      Serial.println("Próba połączenia: wysyłam 'command;'...");
      
      // Czyścimy bufor przed wysłaniem, żeby nie było śmieci
      while(Serial1.available()) Serial1.read();
      
      Serial1.print("command;"); 
    }
  }

  // 2. Obsługa odpowiedzi od RoboMastera
  if (Serial1.available()) {
    String response = Serial1.readStringUntil(';'); // Czytaj do średnika
    
    // Pokaż co przyszło na konsoli PC
    Serial.print("S1 odpisal: ");
    Serial.println(response);

    // Sprawdź czy to potwierdzenie wejścia w tryb SDK
    // Zgodnie z protocal_parser.py robot odpowie "ok" LUB "Already in SDK mode"
    if (response.indexOf("ok") != -1 || response.indexOf("Already in SDK mode") != -1) {
      if (!isConnected) {
        isConnected = true;
        Serial.println(">>> SUKCES: Połączono z RoboMaster S1! <<<");
        
        // Opcjonalnie: Mrugnij diodą lub wydaj dźwięk
        // Tu możesz wywołać funkcję inicjalizującą, np. ustawienie koloru LED
        delay(500);
        Serial1.print("led control comp all r 0 g 255 b 0 effect solid;"); // Zielony LED na znak
      }
    }
  }

  // 3. Pozwól na ręczne wpisywanie komend z PC (dla testów)
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
  }
  
  // 4. Tutaj wstaw swój kod sterujący, który ma działać PO połączeniu
  if (isConnected) {
    // Np. autonomiczne sterowanie:
    // autonomousDrive(); 
  }
}

// Przykładowa funkcja autonomiczna (nieblokująca)
void autonomousDrive() {
  // Tutaj logika sterowania, np. odczyt czujników z Shielda
}