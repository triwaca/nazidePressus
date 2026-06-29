/*
  -  CONTROLE DISPLAY 16X2 COM ESP32  -
  =================================================
  === BLOG DA ROBOTICA - www.blogdarobotica.com ===
  =================================================
  Autor: Jonas Souza
  E-mail: contato@blogdarobotica.com
  Facebook: facebook.com/blogdarobotica
  Instagram:@blogdarobotica
  YouTube: youtube.com/user/blogdarobotica
  =================================================
  === CASA DA ROBOTICA - www.casadarobotica.com ===
  =================================================
  Facebook: facebook.com/casadaroboticaoficial
  Instagram:@casadarobotica
  ==================================================


  https://www.blogdarobotica.com/2022/12/23/como-utilizar-o-display-lcd-16x02-com-modulo-i2c-na-esp32/


  o display tem que estar alimentado com 5V já o sinal poder ser o do esp
  tem pinos predefinidos, 21 e 22, tem que ver qual e o cloc e qual e o sdc
*/

#include <Wire.h> //Biblioteca utilizada gerenciar a comunicação entre dispositicos através do protocolo I2C
#include <LiquidCrystal_I2C.h> //Biblioteca controlar display 16x2 através do I2C

#define col  16 //Define o número de colunas do display utilizado
#define lin   2 //Define o número de linhas do display utilizado
// #define ende  0x3F //Define o endereço do display
#define ende 0x27

LiquidCrystal_I2C lcd(ende,16,2); //Cria o objeto lcd passando como parâmetros o endereço, o nº de colunas e o nº de linhas

void setup() {
  lcd.init(); //Inicializa a comunicação com o display já conectado
  lcd.clear(); //Limpa a tela do display
  lcd.backlight(); //Aciona a luz de fundo do display


}

void loop() {
lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("BLOG DA ROBOTICA"); //Exibe a mensagem na primeira linha do display
  lcd.setCursor(0, 1); //Coloca o cursor do display na coluna 1 e linha 2
  lcd.print("TUTORIAL DISPLAY");  //Exibe a mensagem na segunda linha do display
}
