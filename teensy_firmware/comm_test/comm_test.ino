// -------------------------------------------------------------
// AVTECH AGV - HC-SR04 HARİTALAMA (AKILLI HAFIZA VERSİYONU)
// -------------------------------------------------------------

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int MAX_DISTANCE_CM = 400; 
const unsigned long TIMEOUT_US = MAX_DISTANCE_CM * 58;

// Hata düzeltme hafıza değişkenleri
float sonGecerliMesafe = 0.0;
int hataSayaci = 0;

void setup() {
  Serial.begin(115200); 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  float distance = getDistanceRaw();

  // Sensör geçerli ve mantıklı bir değer okudu mu? (> 2cm ve < 400cm)
  if (distance > 2.0 && distance <= MAX_DISTANCE_CM) {
    sonGecerliMesafe = distance; // Değeri hafızaya al
    hataSayaci = 0;              // Hatayı sıfırla

    Serial.print("Mesafe_cm:");
    Serial.println(distance);
  } 
  else {
    // Sensör "0" okudu (yankıyı kaçırdı veya engel bitti)
    hataSayaci++; 

    // Eğer sadece 1-2 kere kaçırdıysa, grafiği bozmamak için HAFIZADAKİ değeri ver
    if (hataSayaci < 3) {
      Serial.print("Mesafe_cm:");
      Serial.println(sonGecerliMesafe);
    } 
    // Eğer 3 kere ÜST ÜSTE 0 geldiyse, evet engel gerçekten bitti demektir!
    else {
      sonGecerliMesafe = 0.0;
      Serial.println("Mesafe_cm:0");
      hataSayaci = 3; // Sayacın sonsuza gitmesini engelle
    }
  }

  // Sensörün sağlıklı yankı yakalaması için ideal bekleme süresi
  delay(40); 
}

// Temel Okuma Fonksiyonu (Filtreyi yukarıdaki akıllı hafızaya devrettik)
float getDistanceRaw() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Klon sensörlerin uyanması için tetiklemeyi 15 mikro-saniyeye çıkardım
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15); 
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);

  if (duration == 0) return 0.0; 
  return duration / 58.0;
}
