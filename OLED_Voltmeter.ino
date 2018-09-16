/*
 * 2Ch Voltmeter OLED display
 * requires U8g2 library by oliver
 * https://github.com/olikraus/u8g2/wiki/
 * 
 * Wiring:
 * OLED -> Arduino Nano
 * GND -> GND
 * VCC -> 5V
 * SDA -> A4
 * SCL -> A5
 * 
 * Analog:
 * Voltage divider: 62k / 4.7k
 * Reference set to INTERNAL (1.1V)
 * House Battery: A1
 * Engine Battery: A2
 * 
 */


#include <Arduino.h>
//#include <SPI.h>

#include <U8g2lib.h>
/* Constructor for graphic mode */
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(/* rotation=*/ U8G2_R0, /* clock=*/ 21, /* data=*/ 20);

//#include <LowPower.h>

#define HOUSE_SYMBOL 68
#define ENGINE_SYMBOL 79

#define FIRST_ANALOG_PIN A1
#define HOUSE_ANALOG_PIN A1
#define ENGINE_ANALOG_PIN A2


const double ANALOG_FACTOR[] = { (15.2 / 1024.0), (15.2 / 1024.0) };

#define FILTER_QTY 2
#define FILTER_SIZE 10
int filter[FILTER_QTY][FILTER_SIZE];

double house_voltage, engine_voltage;

long next_display_update;
#define DISPLAY_UPDATE_INTERVAL 1000
bool displayAlternate = false;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Started ....");
  u8g2.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);   // LED OFF

  analogReference(INTERNAL);

  // preset all filters
  int f;
  for (int i=0; i<FILTER_SIZE; i++) {
    for (f=0; f<FILTER_QTY; f++) {
      filter[f][i] = 0;
    }
  }
  next_display_update = 0;
}

void display_V(float voltage, char symbol) {
  String displayStr = String(voltage, 1) + "V";
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    String str = String("") + symbol;
    u8g2.drawStr(2,26,str.c_str() );
    u8g2.setFont(u8g2_font_logisoso32_tr); //u8g2_font_ncenB14_tr   u8g2_font_inb30_mn
    u8g2.drawStr(25,32,displayStr.c_str());
    
  } while ( u8g2.nextPage() );

}

void display_str(const char* str) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso32_tr);
    u8g2.drawStr(0,32,str);
  } while ( u8g2.nextPage() );
}

/*
 * Filter analog signal
 * the filter speed is determined by the call frequency of this function
 * and the filter length.
 * The filter is based on an averaged FIFO 
 */
int analog_filter(int *filter_array, int newValue) {
  for (int i=FILTER_SIZE-2; i>=0; i--) {
    filter_array[i+1] = filter_array[i];
  }
  filter_array[0] = newValue;
  int total = 0;
  for (int i=0; i<FILTER_SIZE; i++) {
    total += filter_array[i];
  }
  return total / FILTER_SIZE;
}

double analog_read(byte analog_pin) {
  int ai_raw = analogRead(analog_pin);
  int ai_filtered = analog_filter(filter[analog_pin-FIRST_ANALOG_PIN], ai_raw);
  double voltage = (float)ai_filtered * ANALOG_FACTOR[analog_pin-FIRST_ANALOG_PIN];
  return voltage;
}

void display_update() {
  double voltage;
  if (displayAlternate) {
    display_V( house_voltage, char(HOUSE_SYMBOL) ) ;   
  } else {
    display_V( engine_voltage, char(ENGINE_SYMBOL) );
  }
  displayAlternate = !displayAlternate;
}

void loop(void)
{ 
  //display_V(12.5, char(HOUSE_SYMBOL) );
  house_voltage = analog_read(HOUSE_ANALOG_PIN);
  delay(25);
  engine_voltage = analog_read(ENGINE_ANALOG_PIN);
  delay(25);
  if (millis() > next_display_update) {
    display_update();
    next_display_update = millis() + DISPLAY_UPDATE_INTERVAL; 
  }
  

  
  //LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);

  /*
  delay(1000);


  //display_V(13.8, char(ENGINE_SYMBOL) );
  analog_read(ENGINE_ANALOG_PIN);

  //LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  delay(1000);

  */
}
