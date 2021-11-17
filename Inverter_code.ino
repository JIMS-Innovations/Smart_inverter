#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


int i=0;
int x=0;
int OK=0;
int sinPWM[]={1,2,5,7,10,12,15,17,19,22,24,27,30,32,34,37,39,42,
44,47,49,52,54,57,59,61,64,66,69,71,73,76,78,80,83,
101,103,106,108,110,113,115,117,119,121,124,126,
148,150,152,154,156,158,160,162,164,166,168,169,
195,196,198,199,201,202,204,205,207,208,209,211,
228,229,230,231,232,233,234,235,236,237,237,238,
247,247,247,248,248,248,248,249,249,249,249,249,
248,248,248,247,247,247,246,246,245,245,244,244,
233,232,231,230,229,228,227,226,225,224,223,222,
202,201,199,198,196,195,193,192,190,188,187,185,
158,156,154,152,150,148,146,144,142,140,138,136,
103,101,99,97,94,92,90,88,85,83,80,78,76,73,71,69,
27,24,22,19,17,15,12,10,7,5,2,1};
void setup() {
Serial.begin(9600);
pinMode(5, OUTPUT);
pinMode(6,OUTPUT);
pinMode(9,INPUT);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

}
void inv_off(){
  
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  }
void inv_on(){
  cli();// stop interrupts
TCCR0A=0;//reset the value
TCCR0B=0;//reset the value
TCNT0=0;//reset the value
//0b allow me to write bits in binary
TCCR0A=0b10100001;//phase correct pwm mode
TCCR0B=0b00000001; //no prescaler
TCCR1A=0;//reset the value
TCCR1B=0;//reset the value
TCNT1=0;//reset the value
OCR1A=509;// compare match value
TCCR1B=0b00001001; //WGM12 bit is 1 and no prescaler
TIMSK1 |=(1 << OCIE1A);
sei();// enable interrupts
}
ISR(TIMER1_COMPA_vect){// interrupt when timer 1 match with OCR1A value
if(i>313 && OK==0){// final value from vector for pin 6
i=0;// go to first value of vector
OK=1;//enable pin 5
}
if(i>313 && OK==1){// final value from vector for pin 5
i=0;//go to firs value of vector
OK=0;//enable pin 6
}
x=sinPWM[i];// x take the value from vector corresponding to position i(i is zero indexed)
i=i+1;// go to the next position
if(OK==0){
OCR0B=0;//make pin 5 0
OCR0A=x;//enable pin 6 to corresponding duty cycle
}
if(OK==1){
OCR0A=0;//make pin 6 0
OCR0B=x;//enable pin 5 to corresponding duty cycle
  }
}

void loop() {
if(digitalRead(9) == HIGH){
   display.clearDisplay();

  display.setTextSize(3);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("INV ON"));
  display.display();
  inv_on();
  }
else{
  
   display.clearDisplay();

  display.setTextSize(3);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("INV OFF"));
  display.display();
  inv_off();
  }
}
