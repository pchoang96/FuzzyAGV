#include <Fuzzy.h>
#define clkw        0
#define c_clkw      1

#define encodPinA1  3
#define M1_p        6
#define M1_l        7
#define encodPinA2  2
#define M2_p        5
#define M2_l        4
#define DEBUG

enum fuzzyIn {
    SNB = -60,
    NB  = -50,
    NN  = -25,
    NS  = -5,
    Z   = 0,
    PS  = 5,
    PN  = 25,
    PB  = 50,
    SPB = 60
  };

enum fuzzyOut {
    NM = -90 ,
    NG = -30 , 
    NL = -10,
    NO = 0,
    PL = 10,
    PG = 30,
    PM = 90
  };
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
volatile double ang_vel=0,lin_vel=0;
double l_v,l_vt;
double r_v,r_vt; // pwm: pwm output. lv: mm/sec. lvt: tic/delta_t l:lert, r: right 
bool l_dir=clkw, r_dir=clkw;
/**-----------------------pid velocity calculation-------------------------------------------**/
volatile double  l_error=0.0,l_pre_error=0.0,l_out,l_set,l_ms;
volatile double  r_error=0.0,r_pre_error=0.0,r_out,r_set,r_ms;
/**--------------------------car parameter-----------------------------------------------**/
const double pi=3.1415;
const double sampletime = 0.02, inv_sampletime = 1/sampletime,timer_set=65535-sampletime*250000;
const double wheels_distance = 207, wheels_radius = 31, wheels_diameter=62,wheels_encoder = 440 ;// mm
const double wheel_ticLength = wheels_diameter*pi/wheels_encoder;
int l_p=0,r_p=0,out=0;
double l_d=0,r_d=0;
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
// Instantiating a Fuzzy object
Fuzzy *fuzzy = new Fuzzy();

void setup(){
  //---------------------------------setup hard ware------------------------------------------------
  Serial.begin(9600);
  pinMode(M1_l,OUTPUT);
  pinMode(M2_l,OUTPUT);
  pinMode(encodPinA1, INPUT_PULLUP);                  // encoder input pin
  pinMode(encodPinA2, INPUT_PULLUP);
  attachInterrupt(0, encoder_1 , FALLING);               // update encoder position
  attachInterrupt(1, encoder_2 , FALLING);
  //---------------------------------set up fuzzy algorithm-------------------------------
  /*fuzzy object*/
  FuzzyInput *vel_error = new FuzzyInput(1);
  /*Fuzzy members function*/
  /*Nbig -- Nnormal --Nsmall -- Zero -- Psmall -- Pnormal -- Pbig */
  FuzzySet *Nbig = new FuzzySet(SNB,SNB,NB,NN);
  vel_error->addFuzzySet(Nbig);

  FuzzySet *Nnormal = new FuzzySet(NB,NN,NN,NS);
  vel_error->addFuzzySet(Nnormal);

  FuzzySet *Nsmall = new FuzzySet(NN,NS,NS,Z);
  vel_error->addFuzzySet(Nsmall);

  FuzzySet *Zero = new FuzzySet(NS,Z,Z,PS);
  vel_error->addFuzzySet(Zero);

  FuzzySet *Psmall = new FuzzySet(Z,PS,PS,PN);
  vel_error->addFuzzySet(Psmall);
  
  FuzzySet *Pnormal = new FuzzySet(PS,PN,PN,PB);
  vel_error->addFuzzySet(Pnormal);
  
  FuzzySet *Pbig = new FuzzySet(PN,PB,SPB,SPB);
  vel_error->addFuzzySet(Nbig);
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyInput(vel_error);
//----------------------------------------------------------------------------
  /*fuzzy object*/
  FuzzyOutput *add_speed = new FuzzyOutput(1);
  /*Fuzzy members function*/
  /*Nbig -- Nnormal --Nsmall -- Zero -- Psmall -- Pnormal -- Pbig */
  FuzzySet *Nmuch = new FuzzySet(NM-10,NM-10,NM,NG);
  add_speed->addFuzzySet(Nmuch);

  FuzzySet *Ngay = new FuzzySet(NM,NG,NG,NL);
  add_speed->addFuzzySet(Ngay);

  FuzzySet *Nless = new FuzzySet(NG,NL,NL,NO);
  add_speed->addFuzzySet(Nless);

  FuzzySet *No = new FuzzySet(NL,NO,NO,PL);
  add_speed->addFuzzySet(No);

  FuzzySet *Pless = new FuzzySet(NO,PL,PL,PG);
  add_speed->addFuzzySet(Pless);
  
  FuzzySet *Pgay = new FuzzySet(PL,PG,PG,PM);
  add_speed->addFuzzySet(Pgay);
  
  FuzzySet *Pmuch = new FuzzySet(PG,PM,PM+10,PM+10);
  add_speed->addFuzzySet(Pmuch);
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyOutput(add_speed);
/*----------------------building rules-------------------------------------------------*/
  // Building FuzzyRule "IF error = Nbig THEN add_speed = Nmuch"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Nbig = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Nbig->joinSingle(Nbig);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_Nmuch = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_Nmuch->addOutput(Nmuch);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifError_Nbig, thenAdd_Nmuch);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule01);
  
  // Building FuzzyRule "IF error = Nnormal THEN add_speed = Ngay"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Nnormal = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Nnormal->joinSingle(Nnormal);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_Ngay = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_Ngay->addOutput(Ngay);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ifError_Nnormal, thenAdd_Ngay);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule02);

  // Building FuzzyRule "IF error = Nsmall THEN add_speed = Nless"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Nsmall = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Nsmall->joinSingle(Nsmall);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_Nless = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_Nless->addOutput(Nless);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ifError_Nsmall, thenAdd_Nless);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule03);

    // Building FuzzyRule "IF error = Zero THEN add_speed = No"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Zero = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Zero->joinSingle(Zero);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_No = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_No->addOutput(No);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule04 = new FuzzyRule(4, ifError_Zero, thenAdd_No);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule04);

    // Building FuzzyRule "IF error = Psmall THEN add_speed = Pless"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Psmall = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Psmall->joinSingle(Psmall);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_Pless = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_Pless->addOutput(Pless);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule05 = new FuzzyRule(5, ifError_Psmall, thenAdd_Pless);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule05);

    // Building FuzzyRule "IF error = Pnormal THEN add_speed = Pgay"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Pnormal = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Pnormal->joinSingle(Pnormal);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_Pgay = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_Pgay->addOutput(Pgay);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule06 = new FuzzyRule(6, ifError_Pnormal, thenAdd_Pgay);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule06);

  // Building FuzzyRule "IF error = Pbig THEN add_speed = Pmuch"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ifError_Pbig = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ifError_Pbig->joinSingle(Pbig);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenAdd_Pmuch = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenAdd_Pmuch->addOutput(Pmuch);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule07 = new FuzzyRule(7, ifError_Pbig, thenAdd_Pmuch);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule07);
  //---------------------------------setup timer------------------------------------------ 
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64 4us per pulse
  TCNT1 = timer_set;                      //(12500*4)=50ms
  TIMSK1 |= (1 << TOIE1);                 // Overflow interrupt enable 
  sei();                                  // enable all interrupt
  }

void loop() {
  // put your main code here, to run repeatedly:
  lin_vel = 1000;
  delay(200);
  lin_vel = -1500;
    delay(200);
  lin_vel = -300;
    delay(200);
  lin_vel = 500;
    delay(200);
  lin_vel = 0;
    delay(200);
}

void motion(double lin, double phi )
{
  r_v = (2*lin - phi*wheels_distance)/(2.0); //speed of right wheels  
  l_v = (2*lin + phi*wheels_distance)/(2.0);  //speed of left wheels
  //to l_vt and r_vt
  l_vt = l_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  r_vt = r_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  
  if (l_vt>=0) l_dir = clkw;//go ahead
  else l_dir =c_clkw;       //backhead
  if (r_vt>=0) r_dir = clkw;
  else r_dir =c_clkw;
  
  l_set=abs(l_vt);
  r_set=abs(r_vt);
 // if (l_set>30) l_set=30;
  //if (l_set<5 && l_set>0.5) l_set=5 ;
  //if (r_set>30) r_set=30;
  //if (r_set<5 && r_set>0.5) r_set =5;
}
/*-------------------encoder interrupt 1 ---------------------------------------*/
void encoder_1()
{
  if(!l_dir) l_p ++;
  else l_p--;
}
/*----------------------encoder interrupt 2 ------------------------------------*/
void encoder_2()
{  
  if(!r_dir) r_p ++;
  else r_p--;
}
/*--------------------generarte pwm-----------------------------------*/
void pwmOut(int Lpwm, int Rpwm, bool Ldir, bool Rdir)
{
  if(Lpwm==0 && Rpwm==0)
  { 
    analogWrite(M1_p,0); digitalWrite(M1_l,0);
    analogWrite(M2_p,0); digitalWrite(M2_l,0);
  }
  else if(Ldir==c_clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }

  else if(Ldir==clkw && Rdir==clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0) ;  
  }

  else if(Ldir==clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==c_clkw && Rdir==clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }  
}
/**-------------------------------------------------------------**/
ISR(TIMER1_OVF_vect) 
{
  motion(lin_vel,ang_vel);  
  l_error= l_set-abs(l_p);
  r_error= r_set-abs(r_p);
  //---------------------------------------fuzzy left wheel------------
  fuzzy->setInput(1,l_error);
  fuzzy->fuzzify();
  out = fuzzy->defuzzify(1);
  l_out +=out;
  Serial.println((String) "l_error: " + l_error + " out: " + out + " l_out:" + l_out);
  //---------------------------------------fuzzy right wheel-----------
  fuzzy->setInput(1,r_error);
  fuzzy->fuzzify();
  out = fuzzy->defuzzify(1);
  r_out+=out;
  Serial.println((String) "r_error: " + r_error + " out: " + out + " r_out:" + r_out);
  Serial.println((String) "v_set: " +l_set);
  Serial.println("");
  //---------------------------------------run the car-----------------
  if (l_set==0) {l_out=0; l_dir=c_clkw;}
  if (r_set==0) {r_out=0; r_dir=c_clkw;}
  
  if (l_out>= 255) l_out = 255;
  if (r_out>= 255) r_out = 255;
  pwmOut(l_out,r_out,l_dir,r_dir);
  l_p=0;
  r_p=0;
  TCNT1 =timer_set;
}
/*-----------------------------------------------------------------*/
