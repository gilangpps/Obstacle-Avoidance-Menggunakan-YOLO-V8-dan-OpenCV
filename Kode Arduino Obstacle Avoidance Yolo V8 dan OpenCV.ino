// Pin motor
const int IN1 = 9;  // Motor A input 1
const int IN2 = 8;  // Motor A input 2
const int ENA = 10;  // Motor A enable (PWM)
const int IN3 = 12;  // Motor B input 1
const int IN4 = 13;  // Motor B input 2
const int ENB = 11;  // Motor B enable (PWM)

// Deklarasi fungsi
void moveForward();
void stopMotors();
void turnLeft();
void turnRight();

void setup() {
  // Konfigurasi pin sebagai output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Mulai komunikasi serial
  Serial.begin(9600);
}

void loop() {
  // Cek apakah ada data dari serial
  if (Serial.available()) {
    char command = Serial.read();  // Baca perintah

    // Kendali motor berdasarkan perintah
    if (command == 'F') {  // Forward
      moveForward();
    } else if (command == 'L') {  // Left
      turnLeft();
      delay(850);  // Belok selama 1 detik
      moveForward();  // Kembali maju
    } else if (command == 'R') {  // Right
      turnRight();
      delay(850);  // Belok selama 1 detik
      moveForward();  // Kembali maju
    } else if (command == 'S') {  // Stop
      stopMotors();
    }
  }
}

// Fungsi untuk maju
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);  // Kecepatan motor A
  analogWrite(ENB, 255);  // Kecepatan motor B
}

// Fungsi untuk berhenti
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Fungsi untuk belok kiri
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);  // Kecepatan motor A
  analogWrite(ENB, 255);  // Kecepatan motor B
}

// Fungsi untuk belok kanan
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);  // Kecepatan motor A
  analogWrite(ENB, 255);  // Kecepatan motor B
}
