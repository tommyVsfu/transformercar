
/*7/7 vobotics code version
 *   單純避障右轉 
 */
#include <Ultrasonic.h>
Ultrasonic ultrasonic(5,6);
 
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);  
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
}

void loop() {
  
  double  cc;
  cc = ultrasonic.distanceRead(CM);  // distance in CM
  Serial.print(", CM: ");
  Serial.print(cc);
  if (cc <= 30) {
    right();
    delay(1000);
  }
  else {
    moveforward();  //moveforeward 不能delay 不然他這樣下一次發送超音波會太慢
  }


}
void moveforward() {
digitalWrite(4,HIGH);
digitalWrite(3,LOW);
digitalWrite(8,HIGH);
digitalWrite(7,LOW);
}
void right() {
digitalWrite(4,HIGH);
digitalWrite(3,LOW);
digitalWrite(8,LOW);
digitalWrite(7,HIGH);
}
