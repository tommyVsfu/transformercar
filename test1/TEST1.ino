
/*7/10 vobotics code version
 *    servo + radar
 */
#include <Ultrasonic.h>
#include <Servo.h>
Ultrasonic ultrasonic(5,6); //trigger, echo
Servo myservo; // 建立Servo物件，控制伺服馬達 
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);  
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
myservo.attach(2); // 連接數位腳位9，伺服馬達的訊號線
myservo.write(90);
}

void loop() {
  moveforward();
/*  
// servo 這邊記得每次使用時先確定90的時候超音波頭面向前方，如果沒有就重新插
// myservo.write(90);
 for(int i = 0; i <= 180; i+=1){
    myservo.write(i); // 使用write，傳入角度，從0度轉到180度
    delay(20);
  }
 for(int i = 180; i >= 0; i-=1){
    myservo.write(i);// 使用write，傳入角度，從180度轉到0度
    delay(20);
 }
*/
/*
double cc;
cc = ultrasonic.distanceRead(CM);  // distance in CM
  Serial.print(", CM: ");
  Serial.print(cc);
  if (cc <= 30) { 
  right();
  delay(1000);
  }  
*/  
/*  double  cc;
  cc = ultrasonic.distanceRead(CM);  // distance in CM
  Serial.print(", CM: ");
  Serial.print(cc);
  if (cc <= 30) {
    backward();
    delay(1000);
      if (cc <= 30) {
        whyDontYouFuckOfToBotheringMe();
        delay(1000);
        }
  }   
  else {
    moveforward();  //moveforeward 不能delay 不然他這樣下一次發送超音波會太慢
  }
*/

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
void backward() {
digitalWrite(4,LOW);
digitalWrite(3,HIGH);
digitalWrite(8,LOW);
digitalWrite(7,HIGH);
}
void whyDontYouFuckOfToBotheringMe() {
digitalWrite(4,LOW);
digitalWrite(3,HIGH);
digitalWrite(8,HIGH);
digitalWrite(7,LOW);
}
