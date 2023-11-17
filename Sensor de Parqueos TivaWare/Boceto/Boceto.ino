#define LED 32
#define LDR 12

void setup() {
  pinMode(LDR,INPUT);
  pinMode(LED,OUTPUT);
}

void loop() {
  if(digitalRead(LDR)==HIGH){
    digitalWrite(LED,HIGH);
  }else{
    digitalWrite(LED, LOW);
  }

}
