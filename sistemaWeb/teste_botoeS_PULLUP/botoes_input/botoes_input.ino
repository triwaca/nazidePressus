//#define LED_BUILTIN 2  // LED interno do ESP32
//
//// Pinos dos botões
//const int botoes[] = { 12, 13, 14};
//
//// 2 
//// 12
//// 13
//// 14
//
//
//// Quantidade de piscadas para cada botão
//const int piscadas[] = { 2, 3, 4};
//
//void setup() {
//    pinMode(LED_BUILTIN, OUTPUT);
//
//    for (int i = 0; i < 4; i++) {
//        pinMode(botoes[i], INPUT_PULLUP);  // Configura os botões como entrada com pull-up interno
//    }
//}
//
//void loop() {
//    for (int i = 0; i < 4; i++) {
//        if (digitalRead(botoes[i]) == LOW) {  // Botão pressionado
//            piscarLED(piscadas[i]);  // Pisca o LED o número correspondente de vezes
//            delay(500);  // Pequeno delay para evitar leituras repetidas
//        }
//    }
//}
//
//void piscarLED(int vezes) {
//    for (int i = 0; i < vezes; i++) {
//        digitalWrite(LED_BUILTIN, HIGH);
//        delay(200);
//        digitalWrite(LED_BUILTIN, LOW);
//        delay(200);
//    }
//}


#include <Wire.h> // Biblioteca para comunicação I2C
#include <LiquidCrystal_I2C.h> // Biblioteca para o display LCD 16x2 via I2C

#define COLS 16  // Número de colunas do display
#define LINS 2   // Número de linhas do display
#define ENDE 0x27 // Endereço do display I2C

LiquidCrystal_I2C lcd(ENDE, COLS, LINS); // Objeto do display

// Pinos dos botões (usando GPIOs 2, 12, 13 e 14)
const int botoes[] = {12, 13, 14, 26};

//14 seta esquerda

//13 seta direita

//12 play

//26 circulo


//disposição no painel
// (12)    (26)

// (14)    (13)

void setup() {
    lcd.init();    // Inicializa o display
    lcd.clear();   // Limpa a tela
    lcd.backlight(); // Acende o backlight do LCD

    for (int i = 0; i < 4; i++) {
        pinMode(botoes[i], INPUT_PULLUP);  // Configura os botões como entrada com pull-up interno
    }

    lcd.setCursor(0, 0);
    lcd.print("Aguardando tecla...");
}

void loop() {
    for (int i = 0; i < 4; i++) {
        if (digitalRead(botoes[i]) == LOW) {  // Botão pressionado
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Botao Pressionado:");
            
            lcd.setCursor(0, 1);
            lcd.print("GPIO ");
            lcd.print(botoes[i]);  // Mostra o número do pino pressionado

            delay(500);  // Pequeno delay para evitar leituras repetidas
        }
    }
}
