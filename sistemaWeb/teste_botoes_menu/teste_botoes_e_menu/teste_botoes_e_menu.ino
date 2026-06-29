//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//
//#define COLS 16
//#define LINS 2
//#define ENDE 0x27
//
//LiquidCrystal_I2C lcd(ENDE, COLS, LINS);
//
//const int botoes[] = {12, 13, 14, 26};
//// 14 -> seta esquerda
//// 13 -> seta direita
//// 12 -> play
//// 26 -> circulo
//
//String modos[] = {"Endurance", "Repetition", "Fast"};
//int modoIndex = 0;
//int nivel = 0;
//bool mostrandoNivel = false;
//
//void functionHandle(String modo, String valorStr, String nivelStr) {
//    int valor = valorStr.toInt();
//    int nivel = nivelStr.toInt();
//
//    if (modo == "2") {
//        endurance(valor, nivel);
//    } else if (modo == "3") {
//        repetition(valor, nivel);
//    } else if (modo == "4") {
//        fast(valor, nivel);
//    }
//}
//
//void setup() {
//    lcd.init();
//    lcd.clear();
//    lcd.backlight();
//    
//    for (int i = 0; i < 4; i++) {
//        pinMode(botoes[i], INPUT_PULLUP);
//    }
//
//    mostrarModo();
//}
//
//void loop() {
//    if (!mostrandoNivel) {
//        if (digitalRead(14) == LOW) { // Seta esquerda
//            modoIndex = (modoIndex + 2) % 3;
//            mostrarModo();
//        } else if (digitalRead(13) == LOW) { // Seta direita
//            modoIndex = (modoIndex + 1) % 3;
//            mostrarModo();
//        } else if (digitalRead(12) == LOW) { // Play
//            mostrandoNivel = true;
//            mostrarNivel();
//        }
//    } else {
//        if (digitalRead(14) == LOW) { // Seta esquerda (diminuir nível)
//            if (nivel > 0) nivel--;
//            mostrarNivel();
//        } else if (digitalRead(13) == LOW) { // Seta direita (aumentar nível)
//            if (nivel < 5) nivel++;
//            mostrarNivel();
//        } else if (digitalRead(26) == LOW) { // Círculo (voltar ao modo)
//            mostrandoNivel = false;
//            mostrarModo();
//        } else if (digitalRead(12) == LOW) { // Play (confirmar nível e chamar função)
//            functionHandle(String(modoIndex + 2), "10", String(nivel));
//        }
//    }
//}
//
//void mostrarModo() {
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print(modos[modoIndex]);
//}
//
//void mostrarNivel() {
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print("Nivel: ");
//    lcd.setCursor(7, 0);
//    lcd.print(nivel);
//}
//
//void endurance(int valor, int nivel) {
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print("Endurance");
//    lcd.setCursor(0, 1);
//    lcd.print("V:");
//    lcd.print(valor);
//    lcd.print(" N:");
//    lcd.print(nivel);
//}
//
//void repetition(int valor, int nivel) {
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print("Repetition");
//    lcd.setCursor(0, 1);
//    lcd.print("V:");
//    lcd.print(valor);
//    lcd.print(" N:");
//    lcd.print(nivel);
//}
//
//void fast(int valor, int nivel) {
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print("Fast");
//    lcd.setCursor(0, 1);
//    lcd.print("V:");
//    lcd.print(valor);
//    lcd.print(" N:");
//    lcd.print(nivel);
//}

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define COLS 16
#define LINS 2
#define ENDE 0x27

LiquidCrystal_I2C lcd(ENDE, COLS, LINS);

const int botoes[] = {12, 13, 14, 26};  // Definição dos botões
const int debounceTime = 200; // Tempo de debounce em ms
unsigned long lastPressTime[4] = {0, 0, 0, 0}; // Armazena o tempo da última ativação dos botões

String modos[] = {"Endurance", "Repetition", "Fast"};
int modoIndex = 0;
int nivel = 0;
bool mostrandoNivel = false;

void functionHandle(String modo, String valorStr, String nivelStr) {
    int valor = valorStr.toInt();
    int nivel = nivelStr.toInt();

    if (modo == "2") {
        endurance(valor, nivel);
    } else if (modo == "3") {
        repetition(valor, nivel);
    } else if (modo == "4") {
        fast(valor, nivel);
    }
}

void setup() {
    lcd.init();
    lcd.clear();
    lcd.backlight();
    
    for (int i = 0; i < 4; i++) {
        pinMode(botoes[i], INPUT_PULLUP);
    }

    mostrarModo();
}

void loop() {
    unsigned long currentTime = millis(); // Captura o tempo atual

    for (int i = 0; i < 4; i++) {
        if (digitalRead(botoes[i]) == LOW) { // Botão pressionado
            if (currentTime - lastPressTime[i] > debounceTime) { // Verifica debounce
                lastPressTime[i] = currentTime; // Atualiza o tempo da última ativação

                if (!mostrandoNivel) {
                    if (botoes[i] == 14) { // Seta esquerda
                        modoIndex = (modoIndex + 2) % 3;
                        mostrarModo();
                    } else if (botoes[i] == 13) { // Seta direita
                        modoIndex = (modoIndex + 1) % 3;
                        mostrarModo();
                    } else if (botoes[i] == 12) { // Play
                        mostrandoNivel = true;
                        mostrarNivel();
                    }
                } else {
                    if (botoes[i] == 14) { // Seta esquerda (diminuir nível)
                        if (nivel > 0) nivel--;
                        mostrarNivel();
                    } else if (botoes[i] == 13) { // Seta direita (aumentar nível)
                        if (nivel < 5) nivel++;
                        mostrarNivel();
                    } else if (botoes[i] == 26) { // Círculo (voltar ao modo)
                        mostrandoNivel = false;
                        mostrarModo();
                    } else if (botoes[i] == 12) { // Play (confirmar nível e chamar função)
                        functionHandle(String(modoIndex + 2), "10", String(nivel));
                    }
                }
            }
        }
    }
}

void mostrarModo() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(modos[modoIndex]);
}

void mostrarNivel() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Nivel: ");
    lcd.setCursor(7, 0);
    lcd.print(nivel);
}

void endurance(int valor, int nivel) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Endurance");
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(valor);
    lcd.print(" N:");
    lcd.print(nivel);
}

void repetition(int valor, int nivel) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Repetition");
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(valor);
    lcd.print(" N:");
    lcd.print(nivel);
}

void fast(int valor, int nivel) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Fast");
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(valor);
    lcd.print(" N:");
    lcd.print(nivel);
}
