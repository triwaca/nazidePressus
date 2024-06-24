#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define stepPinX 5 // motor X
#define dirPinX 2

#define stepPinY 6 // motor Y
#define dirPinY 3

// AccelStepper Setup
AccelStepper stepperX(1, stepPinX, dirPinX); // motor X
AccelStepper stepperY(1, stepPinY, dirPinY);  // motor Y



// Define the Pins used for homing switches
#define home_switchX 10 // Pin 10 home do motor X
#define home_switchY 11 // Pin 11 home do motor Y

// Stepper Travel Variables
long initial_homingX = -1;  // Used to Home Stepper X at startup
long initial_homingY = -1;  // Used to Home Stepper Y at startup



#define ok_pin 12  // Pino do botão          // Variável para armazenar o contador
#define previous_pin 4
#define next_pin 9

bool but_ok;           // Variável para armazenar o estado do botão
bool lastBut_ok = HIGH; // Variável para armazenar o último estado do botão
bool but_previous;
bool lastBut_previous = HIGH;
bool but_next;
bool lastBut_next = HIGH;

unsigned long lastDebounceTime = 0;  // Último tempo de debounce
unsigned long debounceDelay = 50;    // Tempo de debounce (50 milliseconds)
unsigned long lastPressTime = 0;     // Último tempo que o botão foi pressionado

int velocidadeMovimentoX = 10000000;  // configura a velocidade dos movimetos
int velocidadeMovimentoY = 10000000;  // configura a velocidade dos movimetos
int lento = 5000;     // configura a velocidade do motor (ciclo de movimento)
int rapido = 500;     // configura a velocidade do motor (ciclo de movimento)
//int tempoMovimento = 5; // Configura o total de movimentos essa variavel depende da velocidade do motor

bool flag = true;

byte choice = 0;


void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);

  pinMode(home_switchX, INPUT_PULLUP);
  pinMode(home_switchY, INPUT_PULLUP);
  pinMode(ok_pin, INPUT_PULLUP); // Define o pino do botão como entrada com pull-up interno
  pinMode(previous_pin, INPUT_PULLUP);
  pinMode(next_pin, INPUT_PULLUP);
  delay(5);

  stepperX.setMaxSpeed(10000000);
  stepperX.setAcceleration(10000000);

  stepperY.setMaxSpeed(10000000);
  stepperY.setAcceleration(10000000);

  while (digitalRead(home_switchX)) {
    stepperX.move(initial_homingX);
    stepperX.run();
  }

  stepperX.setCurrentPosition(0);
  initial_homingX = 1;

  while (!digitalRead(home_switchX)) {
    stepperX.move(initial_homingX);
    stepperX.run();
  }
  stepperX.setCurrentPosition(0);
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("Motor X:  OK");

  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Motor Y: Calibrando");

  while (digitalRead(home_switchY)) {
    stepperY.move(initial_homingY);
    stepperY.run();
  }

  stepperY.setCurrentPosition(0);
  initial_homingY = 1;

  while (!digitalRead(home_switchY)) {
    stepperY.move(initial_homingY);
    stepperY.run();
  }
  stepperY.setCurrentPosition(0);

  lcd.setCursor(0, 1);
  lcd.clear();
  lcd.print("Motor Y:  OK");

  // Devinição de velocidades
  stepperX.setMaxSpeed(velocidadeMovimentoX);
  stepperX.setAcceleration(velocidadeMovimentoX);
  stepperY.setMaxSpeed(velocidadeMovimentoY);
  stepperY.setAcceleration(velocidadeMovimentoY);


  delay(2000);

  // CALIBRAÇÃO DO ZERO
  stepperX.moveTo(-22 * 32);  //
  stepperY.moveTo(-22 * 32);  //
  stepperX.runToPosition();   // Espera até que o movimento seja concluído
  stepperY.runToPosition();   // Espera até que o movimento seja concluído

  stepperX.setCurrentPosition(0); // ZERO ABSOLUTO
  stepperY.setCurrentPosition(0); // ZERO ABSOLUTO

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready engines!");
  delay(2000); // Espera 2 segundo
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Now: NIVEL0");
  lcd.setCursor(0, 1);
  lcd.print("     NIVEL0    >");
}

bool debounceButton(int pin, bool &buttonState, bool &lastButtonState) {
  static unsigned long lastDebounceTime = 0;  // Último tempo de debounce (faz local à função)
  int reading = digitalRead(pin);

  // Se o estado lido é diferente do último estado, reinicia o contador de debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Se o tempo desde a última mudança é maior que o intervalo de debounce
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // E se o estado lido for diferente do estado atual do botão
    if (reading != buttonState) {
      buttonState = reading;  // Atualiza o estado do botão
    }
  }

  // Atualiza o último estado lido
  lastButtonState = reading;

  return buttonState;  // Retorna o estado atual do botão
}
void fast5() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 4      ");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 100;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 4     ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 4     ");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 125 * 32;
    int posY = 125 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 140 * 32 ? 125 * 32 : 140 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 140 * 32 ? 125 * 32 : 140 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= rapido) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(120 * 32);
    stepperY.moveTo(120 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 100;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 10;
  }
  else if (!but_next) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 4     ");
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 4     ");
    choice = 100;

  }
}

void contracao5() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("< REPETITION 4 >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 10;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now:REPETITION 4");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now:REPETITION 4");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 125 * 32;
    int posY = 125 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 140 * 32 ? 125 * 32 : 140 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 140 * 32 ? 125 * 32 : 140 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= lento) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(120 * 32);
    stepperY.moveTo(120 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 10;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 9;
  }
  else if (!but_next) {
    flag = true;
    choice = 100;
  }
}

void nivel5() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<  ENDURANCE4  >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 9;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 4");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 4");


    stepperX.moveTo(140 * 32);
    stepperY.moveTo(140 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 9;
  }
  else if (!but_previous) {
    flag = true;
    choice = 80;
  }
  else if (!but_next) {
    flag = true;
    choice = 10;
  }
}
void fast4() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 5    >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 80;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 5     ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 5     ");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 160 * 32;
    int posY = 160 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 145 * 32 ? 160 * 32 : 145 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 145 * 32 ? 160 * 32 : 145 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= rapido) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(120 * 32);
    stepperY.moveTo(120 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 80;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 8;
  }
  else if (!but_next) {
    flag = true;
    choice = 9;
  }
}

void contracao4() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("< REPETITION 5 >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 8;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: REPETITION5");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: REPETITION5");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 160 * 32;
    int posY = 160 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 145 * 32 ? 160 * 32 : 145 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 145 * 32 ? 160 * 32 : 145 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= lento) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(120 * 32);
    stepperY.moveTo(120 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 8;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 7;
  }
  else if (!but_next) {
    flag = true;
    choice = 80;
  }
}

void nivel4() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<  ENDURANCE5  >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 7;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 5");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 5");


    stepperX.moveTo(145 * 32);
    stepperY.moveTo(145 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 7;
  }
  else if (!but_previous) {
    flag = true;
    choice = 60;
  }
  else if (!but_next) {
    flag = true;
    choice = 8;
  }
}
void fast3() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 3    >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 60;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 3     ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 3     ");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 100 * 32;
    int posY = 100 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 80 * 32 ? 100 * 32 : 80 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 80 * 32 ? 100 * 32 : 80 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= rapido) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(120 * 32);
    stepperY.moveTo(120 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 60;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 6;
  }
  else if (!but_next) {
    flag = true;
    choice = 7;
  }
}

void contracao3() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("< REPETITION 3 >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 6;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now:REPETITION 3");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now:REPETITION 3");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 100 * 32;
    int posY = 100 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 80 * 32 ? 100 * 32 : 80 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 80 * 32 ? 100 * 32 : 80 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= lento) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(120 * 32);
    stepperY.moveTo(120 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 6;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 5;
  }
  else if (!but_next) {
    flag = true;
    choice = 60;
  }
}

void nivel3() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<  ENDURANCE3  >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 5;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 3");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 3");

    stepperX.moveTo(100 * 32);
    stepperY.moveTo(100 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 5;
  }
  else if (!but_previous) {
    flag = true;
    choice = 40;
  }
  else if (!but_next) {
    flag = true;
    choice = 6;
  }
}
void fast2() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 2    >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 40;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("<    FAST 2    >");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 2     ");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 60 * 32;
    int posY = 60 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 40 * 32 ? 60 * 32 : 40 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 40 * 32 ? 60 * 32 : 40 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= rapido) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(80 * 32);
    stepperY.moveTo(80 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 40;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 4;
  }
  else if (!but_next) {
    flag = true;
    choice = 5;
  }
}

void contracao2() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("< REPETITION 2 >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 4;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                 ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now:REPETITION 2");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now:REPETITION 2");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 60 * 32;
    int posY = 60 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 40 * 32 ? 60 * 32 : 40 * 32; // Troca a posição de destino
          posY = stepperY.currentPosition() == 40 * 32 ? 60 * 32 : 40 * 32;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= lento) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(80 * 32);
    stepperY.moveTo(80 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 4;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 3;
  }
  else if (!but_next) {
    flag = true;
    choice = 40;
  }
}


void nivel2() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<  ENDURANCE2  >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 3;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 2");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 2");

    stepperX.moveTo(60 * 32);
    stepperY.moveTo(60 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 3;
  }
  else if (!but_previous) {
    flag = true;
    choice = 20;
  }
  else if (!but_next) {
    flag = true;
    choice = 4;
  }
}
void fast1() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<    FAST 1    >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 20;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 1     ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: FAST 1     ");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 20 * 32;
    int posY = 20 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 0 ? 20 * 32 : 0; // Troca a posição de destino
          posY = stepperY.currentPosition() == 0 ? 20 * 32 : 0;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= rapido) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(40 * 32);
    stepperY.moveTo(40 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 20;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 2;
  }
  else if (!but_next) {
    flag = true;
    choice = 3;
  }
}

void contracao1() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("< REPETITION 1 >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 2;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: REPETITION1");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: REPETITION1");

    unsigned long finishTime = 0;
    bool flag1 = true;
    int posX = 20 * 32;
    int posY = 20 * 32;
    stepperX.moveTo(posX);
    stepperY.moveTo(posY);
    while (digitalRead(ok_pin)) { // Enquanto o botão ok não for pressionado, o motor girará
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {  // Verifica se o motor chegou ao destino
        if (flag1) {
          stepperX.setCurrentPosition(posX);
          stepperY.setCurrentPosition(posY);
          posX = stepperX.currentPosition() == 0 ? 20 * 32 : 0; // Troca a posição de destino
          posY = stepperY.currentPosition() == 0 ? 20 * 32 : 0;
          flag1 = false;  // Reset a flag
          finishTime = millis();  // Atualiza o tempo de referência
        }

        if (millis() - finishTime >= lento) {  // Se o intervalo definido passou
          stepperX.moveTo(posX - 1);
          stepperY.moveTo(posY - 1);
          flag1 = true;  // Seta a flag para mover os motores no próximo ciclo
        }
      }
      else {
        if (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        if (stepperY.distanceToGo() != 0) {
          stepperY.run();
        }
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Now: N/A        ");
    stepperX.moveTo(40 * 32);
    stepperY.moveTo(40 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 2;
    delay(500);
  }
  else if (!but_previous) {
    flag = true;
    choice = 1;
  }
  else if (!but_next) {
    flag = true;
    choice = 20;
  }
}

void nivel1() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("<  ENDURANCE1  >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 1;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 1");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: ENDURANCE 1");

    stepperX.moveTo(20 * 32);
    stepperY.moveTo(20 * 32);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 1;
  }
  else if (!but_previous) {
    flag = true;
    choice = 0;
  }
  else if (!but_next) {
    flag = true;
    choice = 2;
  }
}

void nivel0() {
  but_ok = HIGH;
  but_previous = HIGH;
  but_next = HIGH;
  if (flag) {
    lcd.setCursor(0, 1);
    lcd.print("     NIVEL0    >");
    flag = false;
    delay(100);
  }
  but_ok = debounceButton(ok_pin, but_ok, lastBut_ok);
  but_previous = debounceButton(previous_pin, but_previous, lastBut_previous);
  but_next = debounceButton(next_pin, but_next, lastBut_next);

  if (but_ok && but_previous && but_next) {
    choice = 0;
  }
  else if (!but_ok) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: NIVEL0     ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Now: NIVEL0     ");
    stepperX.moveTo(0);
    stepperY.moveTo(0);
    while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      stepperY.run();
    }
    choice = 0;
  }
  else if (!but_previous) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("     NIVEL0    >");
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("                ");
    delay(100);
    lcd.setCursor(0, 1);
    lcd.print("     NIVEL0    >");
    choice = 0;
  }
  else if (!but_next) {
    flag = true;
    choice = 1;
  }
}

void loop() {
  if (choice == 0) {
    //Serial.println("estou no 0");
    nivel0();
  }
  else if (choice == 1) {
    //Serial.println("estou no 1");
    nivel1();
  }
  else if (choice == 2) {
    //Serial.println("estou no 2");
    contracao1();
  }
  else if (choice == 20) {
    fast1();
  }
  else if (choice == 3) {
    //Serial.println("estou no 3");
    nivel2();
  }
  else if (choice == 4) {
    //Serial.println("estou no 4");
    contracao2();
  }
  else if (choice == 40) {
    fast2();
  }
  else if (choice == 5) {
    //Serial.println("estou no 5");
    nivel3();
  }
  else if (choice == 6) {
    // Serial.println("estou no 6");
    contracao3();
  }
  else if (choice == 60) {
    fast3();
  }
  else if (choice == 7) {
    //Serial.println("estou no 7");
    nivel4();
  }
  else if (choice == 8) {
    //Serial.println("estou no 8");
    contracao4();
  }
  else if (choice == 80) {
    fast4();
  }
  else if (choice == 9) {
    //Serial.println("estou no 9");
    nivel5();
  }
  else if (choice == 10) {
    //Serial.println("estou no 10");
    contracao5();
  }
  else if (choice == 100) {
    fast5();
  }
}
