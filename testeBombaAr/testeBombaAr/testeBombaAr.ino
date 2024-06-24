// Para Arduino Nano

const int A1A = 2;//define pin 2 for A1A
const int A1B = 3;//define pin 3 for A1B

const int B1A = 18;//define pin 8 for B1A
const int B1B = 19;//define pin 9 for B1B

void setup() {
  Serial.begin(9600);
  pinMode(B1A,OUTPUT);
  pinMode(B1B,OUTPUT);
  
  pinMode(A1A,OUTPUT);
  pinMode(A1B,OUTPUT);    
  delay(3000);
  Serial.println("Iniciando...");
}

void loop() {
  ligaCompressor();
  delay(6000);
  desligaCompressor();
  delay(10000);
}

void ligaCompressor(){
  Serial.println("Compressor ligado!");
  motorA('R');
}

void desligaCompressor(){
  Serial.println("Compressor desligado.");
  motorA('O');
}

void motorA(char d){
  if(d =='R'){
    digitalWrite(A1A,LOW);
    digitalWrite(A1B,HIGH); 
  }else if (d =='L'){
    digitalWrite(A1A,HIGH);
    digitalWrite(A1B,LOW);    
  }else{
    digitalWrite(A1A,LOW);
    digitalWrite(A1B,LOW);    
  }
}

void motorB(char d){

    if(d =='R'){
      digitalWrite(B1A,LOW);
      digitalWrite(B1B,HIGH); 
    }else if(d =='L'){
      digitalWrite(B1A,HIGH);
      digitalWrite(B1B,LOW);    
    }else{    
      digitalWrite(B1A,LOW);
      digitalWrite(B1B,LOW);     
    }

}
