/*7/6 vobotics code version
 *    馬達驅動電路，與前進code 完成
 */
void setup() {
  // put your setup code here, to run once:
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
}

void loop() {


  
  // put your main code here, to run repeatedly:
digitalWrite(4,HIGH);
digitalWrite(3,LOW);
digitalWrite(8,HIGH);
digitalWrite(7,LOW);
}
