#define throttle_in 5
#define vertical_in 3
#define horizontal_in 4
#define Switch_in 2


void setup() {
Serial.begin(9600);
pinMode(throttle_in,INPUT);
pinMode(vertical_in,INPUT);
pinMode(horizontal_in,INPUT);
pinMode(Switch_in,INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:
float throttle=analogRead(throttle_in)/1024.0;
float vertical=analogRead(vertical_in)/1024.0;
float horizontal=analogRead(horizontal_in)/1024.0;
int Switch=digitalRead(Switch_in);

Serial.print("S");
Serial.print(Switch);
Serial.print("T");
Serial.print(throttle);
Serial.print("H");
Serial.print(horizontal);
Serial.print("V");
Serial.print(vertical);
Serial.print("E");


}
