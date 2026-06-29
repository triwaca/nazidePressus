//https://www.makerguides.com/esp32-and-tb6600-stepper-motor-driver/
//int PUL = 25; //define Pulse pin
//int DIR = 26; //define Direction pin
//int ENA = 27; //define Enable Pin

int PUL = 33; // Pino de Pulso (Pulse +)
int DIR = 25; // Pino de Direção (Dir +)
int ENA = 32; //define Enable Pin


//void setup() {
//  pinMode (PUL, OUTPUT);
//  pinMode (DIR, OUTPUT);
//  pinMode (ENA, OUTPUT);
//}
//
//void loop() {
//  for (int i = 0; i < 6400; i++) // Forward 5000 steps
//  {
//    digitalWrite(DIR, LOW);
//    digitalWrite(ENA, HIGH);
//    digitalWrite(PUL, HIGH);
//    delayMicroseconds(50);
//    digitalWrite(PUL, LOW);
//    delayMicroseconds(50);
//  }
//  for (int i = 0; i < 6400; i++) // Backward 5000 steps
//  {
//    digitalWrite(DIR, HIGH);
//    digitalWrite(ENA, HIGH);
//    digitalWrite(PUL, HIGH);
//    delayMicroseconds(50);
//    digitalWrite(PUL, LOW);
//    delayMicroseconds(50);
//  }
//}

//teste constante =================================
//
//void setup() {
//  pinMode(PUL, OUTPUT);
//  pinMode(DIR, OUTPUT);
//  pinMode(ENA, OUTPUT);
//  digitalWrite(ENA, LOW); // Tente habilitar em LOW
//  digitalWrite(DIR, LOW); // Defina a direção para frente
//}
//
//void loop() {
//  digitalWrite(PUL, HIGH);
//  delayMicroseconds(100);
//  digitalWrite(PUL, LOW);
//  delayMicroseconds(100);
//}


//
//void setup() {
//  pinMode (PUL, OUTPUT);
//  pinMode (DIR, OUTPUT);
//  pinMode (ENA, OUTPUT);
//}
//
//void loop() {
//  for (int i = 0; i < 6400; i++) // Forward 5000 steps
//  {
//    digitalWrite(DIR, LOW);
//    digitalWrite(ENA, HIGH);
//    digitalWrite(PUL, HIGH);
//    delayMicroseconds(50);
//    digitalWrite(PUL, LOW);
//    delayMicroseconds(50);
//  }
//  for (int i = 0; i < 6400; i++) // Backward 5000 steps
//  {
//    digitalWrite(DIR, HIGH);
//    digitalWrite(ENA, HIGH);
//    digitalWrite(PUL, HIGH);
//    delayMicroseconds(50);
//    digitalWrite(PUL, LOW);
//    delayMicroseconds(50);
//  }
//}


//so com pulso e dir sem enable

//int PUL = 25; // Pino para o pulso
//int DIR = 26; // Pino para a direção
//// int ENA = 27; // Pino para habilitar - REMOVIDO

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  // digitalWrite(ENA, LOW); // REMOVIDO
}

void loop() {
  // Movimento para frente
  digitalWrite(DIR, LOW);
  for (int i = 0; i < 6400; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
  }

  delay(500); // Pequena pausa entre movimentos

  // Movimento para trás
  digitalWrite(DIR, HIGH);
  for (int i = 0; i < 6400; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
  }

  delay(500); // Pausa entre ciclos
}
