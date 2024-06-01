

#include <avr/wdt.h>
#include <PID_v1.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(13, 12, 7, 6, 5, 4);
#define TEPLNAD 0
#define TEPLPOD 1					
#define PODEHREV 8    // Output to Opto Triac pin
#define HOHREV 9    // Output to Opto Triac pin
#define IMPULS 4   //trigger pulse width (counts)
float teplotanad = 0;
float teplotanadPol[5]={0,0,0,0,0};
float teplotapod = 0;			    
unsigned long puvodniMillis = 0;
unsigned long casomira = 0;
int cas = 0;
const int interval = 600;
int horVykon = 0;  // Dimming level (60-600)  60 = ON, 600 = OFF
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpoint2, Input2, Output2;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,18,0.07,60, DIRECT);        //podehřev
PID myPID2(&Input2, &Output2, &Setpoint2,60,1,20, DIRECT);    //horní ohřev 90 2 40 moc agresivní
int WindowSize = 5000;
unsigned long windowStartTime;
int RezimHOhrevu = 0;


void setup()
{
 Serial.begin(115200);
 lcd.begin(16, 4);
 pinMode(2, INPUT);
 pinMode(PODEHREV, OUTPUT);// Set AC Load pin as output
 pinMode(HOHREV, OUTPUT);// Set AC Load pin as output
 //ovladani
  pinMode(A7, INPUT);	  // pot h
  pinMode(A6, INPUT);	   // pot d
  pinMode(A5, INPUT);	   // h
  digitalWrite(A5, HIGH);   // 
  pinMode(A4, INPUT);	   // s
  digitalWrite(A4, HIGH);   // 
  pinMode(A3, INPUT);	   // d
  digitalWrite(A3, HIGH);   // 
  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 100;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled
  // set up zero crossing interrupt
  attachInterrupt(0,preruseniPruchodNulou, RISING);    
    //IRQ0 is pin 2. Call zeroCrossingInterrupt 
    //on rising signal



  windowStartTime = millis();
  
  //initialize the variables we're linked to
  Setpoint = 0;
  Setpoint2 = 0;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  myPID2.SetOutputLimits(0, 10000);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);




 lcd.clear(); 
 lcd.setCursor(0, 0);
 lcd.print("IRpajk v1 cas:");
 lcd.setCursor(0, 1);
 lcd.print("t1s:      t1n: ");
 lcd.setCursor(0, 2);
 lcd.print("t2s:      t2n: ");
 lcd.setCursor(0, 3);
 lcd.print("fr1:      fr2:");
}

//Interrupt Service Routines
void preruseniPruchodNulou(){ //zero cross detect   
  TCCR1B=0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
}
ISR(TIMER1_COMPA_vect){ //comparator match
  if(digitalRead(A3) != 0 && horVykon > 5) {digitalWrite(HOHREV,HIGH);}  //set triac gate to high
  TCNT1 = 65536-IMPULS;      //trigger pulse width
}
ISR(TIMER1_OVF_vect){ //timer1 overflow
  digitalWrite(HOHREV,LOW); //turn off triac gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
  
wdt_enable(WDTO_8S);
}




void PID1()
{
  Input = teplotapod;
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
 if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  //if (Input < Setpoint/1.1) Output = 3000;
  //if (Output > 3000) Output = 3000;
  if((Output) < millis() - windowStartTime)   digitalWrite(PODEHREV,LOW);  else   digitalWrite(PODEHREV,HIGH);
}

void PID2()
{
  Input2 = teplotanad;
  myPID2.Compute();
  
  if (RezimHOhrevu == 1) {
      horVykon = Output2/100;
    }else{
      horVykon = map(Setpoint2, 0, 450, 0, 100);
    }
    
  if(Setpoint2 < 40) horVykon = 0;
  OCR1A = map(horVykon, 0, 100, 600, 60); // 650 slo za roh - plnej kotel
 
}


void zobrazovac ()					 
{
  if(digitalRead(A5) == 0 ) Serial.println(" tl1 ");
  if(digitalRead(A4) == 0 ) Serial.println(" tl2 ");
  if(digitalRead(A3) == 0 ) Serial.println(" tl3 ");
  Serial.print(horVykon);
  Serial.print(" podh: ");
  Serial.print(Output);
  Serial.print(" nadh: ");
  Serial.print(Output2);
  Serial.print(" fz: ");
  Serial.print (horVykon);
  Serial.print(" nad: ");
  Serial.print ((long)teplotanad);	
  Serial.print(" pod: ");
  Serial.println ((long)teplotapod);	 
  Serial.println (!digitalRead(A4));
  lcd.setCursor(5, 1);
  lcd.print((long)teplotanad);	
  lcd.print(" ");
  lcd.setCursor(15, 0); //cas
  cas = (long)((millis()-casomira)/60000);
  lcd.print(cas);
  if(cas<10000) lcd.print("m"); 
  if(cas>=10000) cas = 0;
  if(cas<10) lcd.print(" ");
  if(cas<100) lcd.print(" ");
  if(cas<1000) lcd.print(" ");
  lcd.setCursor(15, 1);
    if (RezimHOhrevu == 1) {
      lcd.print((long)Setpoint2);
      lcd.print(" ");
      if(Setpoint2<10) lcd.print(" ");
    }else{
      lcd.print("MAN");
    }
  lcd.setCursor(5, 2);
  lcd.print((long)teplotapod);
  lcd.print(" ");
  lcd.setCursor(15, 2);
  lcd.print((long)Setpoint);
  lcd.print("  ");  
  if(Setpoint<10) lcd.print(" ");
  lcd.setCursor(5, 3);
  lcd.print(horVykon);
  lcd.print("  ");
  if(digitalRead(PODEHREV) == 0){ 
  lcd.setCursor(15, 3);
  lcd.print("VYP.");    
  }  else {
  lcd.setCursor(15, 3);
  lcd.print("ZAP.");
  }
}

void loop ()					 
{
  wdt_reset();
  unsigned long tedMillis = millis();
  if(tedMillis - puvodniMillis >= interval) {
  puvodniMillis = tedMillis;
  // nacti seriak
  while (Serial.available() > 0) {
 //   horVykon = Serial.parseInt();
 //Setpoint2 = Serial.parseInt();
 Setpoint = Serial.parseInt();
 //   if (horVykon < 0) horVykon = 0;
 //   if (horVykon > 100) horVykon = 100;
 //   OCR1A = map(horVykon, 0, 100, 600, 60);
    if (Serial.read() == '\n') {
    }
  }
  if(digitalRead(A5) == 0) casomira = millis();
  RezimHOhrevu = digitalRead(A4);
  if (RezimHOhrevu == 1) {
      myPID2.SetMode(AUTOMATIC);
    }else{
      myPID2.SetMode(MANUAL);
    }
  if(digitalRead(A3) == 0) { asm volatile ("  jmp 0");  } 
  //potaky
  Setpoint = map(analogRead(A6), 0, 1024, 0, 450);
  Setpoint2 = map(analogRead(A7), 0, 1024, 0, 450);
  
  // mereni teploty
  
//teplotanad = ((analogRead(TEPLNAD)*1.16)+35); // +50 ujizdelo o 25
  teplotanadPol[0] = teplotanadPol[1];
  teplotanadPol[1] = teplotanadPol[2];
  teplotanadPol[2] = teplotanadPol[3];
  teplotanadPol[3] = teplotanadPol[4];
  teplotanadPol[4] = ((analogRead(TEPLNAD)*1.16)+40); // +50 ujizdelo o 25
  teplotanad = (teplotanadPol[1]+teplotanadPol[2]+teplotanadPol[3]+teplotanadPol[4]+teplotanadPol[0]) * 0.2;
  
  // teplotanad presunuto do pidu
  teplotapod = ((analogRead(TEPLPOD)*1.15)+30);
  
  zobrazovac();
  PID2();
  }
  PID1();
}						  




