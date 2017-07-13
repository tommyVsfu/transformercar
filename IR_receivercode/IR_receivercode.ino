/*
 * 紅外線接收器簡易測試:
 *    在按下電視或其它遙控器按扭時讓紅外線指示燈閃爍幾下
 */
#include <IRremote.h>              // 引用 IRRemote 函式庫
const int irReceiver = 11;          // 紅外線接收器
const int ledPin = 13;             // 紅外線指示燈
IRsend irsend;                           // 定義 IRsend 物件來發射紅外線訊號
void setup() {                
  Serial.begin(9600);               // 開啟 Serial port, 通訊速率為 9600 bps
  pinMode(irReceiver, INPUT);      // 把 irReceiver 接腳設置為 INPUT
  pinMode(ledPin, OUTPUT);         // 把 ledPin 設置為 OUTPUT
}



void loop() {
 IR_receiver();

}

void IR_receiver() {
    if (Serial.read() != -1) {    
    // 發射紅外線訊號
    irsend.sendNEC(0x4FB48B7, 32);   // 我電視 Power 鈕的紅外線編碼, 記得換成你的紅外線編碼 
  }
    int ir_status = digitalRead(irReceiver);  // 讀取 irReceiver 的狀態
  // 檢查 irReceiver 是否有收到紅外線訊號
  // 有的話，ir_status 會是 0 (因為 Receiver 會把訊號反向, 所以 0 代表有收到訊號)
  if (ir_status == 0)                      
    blinkLED();                             // 讓指示燈閃爍幾下 
}
// 讓指示燈閃爍幾下 
void blinkLED() {
  for (int i=1; i<= 4; i++) {
    digitalWrite(ledPin, HIGH);     // 打開指示燈
    delay(100);
    digitalWrite(ledPin, LOW);      // 關掉指示燈
    delay(100);
  }  
}
