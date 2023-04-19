
int LED13 = 13;   
int Signal;                
int Threshold = 518;            

void setup() {
  pinMode(LED13,OUTPUT);         
   Serial.begin(9600);         
}

void loop() {
  Signal = analogRead(0);                                  
  Serial.println(Signal);      

   if(Signal > Threshold){               
     digitalWrite(LED13,HIGH);
   } else {
     digitalWrite(LED13,LOW);           
   }
   
  delay(10);
}