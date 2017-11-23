/*working variables*/

//------------------------------------------------------
int NbTopsFan; 
int Calc;
int hallsensor = 2; 
float valor = 0;
//----------------------------------------------------
int sensor_pin = A0;
//int pot = A1;
int pwm = 11;
//----------------------------------------------------
unsigned long lastTime;
double Input, Output;
double errSum, lastErr, lastInput, dErr,error;
double kp, ki, kd;
double Setpoint = 32;
int SampleTime = 1000; //1 sec
//----------------------------------------------------
// rpm -----------------------------------------------
typedef struct{

char fantype;
unsigned int fandiv; }fanspec;
 
void rpm(){ 
NbTopsFan++; 
}
//----------------------------------------------------
String leStringSerial(){
  String conteudo = "";
  char caractere;
  
  // Enquanto receber algo pela serial
  while(Serial.available() > 0) {
    caractere = Serial.read();  // Lê byte da serial
    if (caractere != '\n'){  // Ignora caractere de quebra de linha
      conteudo.concat(caractere);  // Concatena valores
    }
    // Aguarda buffer serial ler próximo caractere 
    delay(10);
  }
    
  Serial.print("Recebi: ");
  Serial.println(conteudo);
    
  return conteudo;
}

//----------------------------------------------------
//----------------------------------------------------
void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
   /*Compute all the working error variables*/
      error = Setpoint - Input;
      errSum += error;
      double dInput = (Input - lastInput);
      
       
       /*Compute PID Output*/
       Output = 120 + (kp * error + ki * errSum - kd * dInput)*(-1);
      
       /*Remember some variables for next time*/
       lastErr = Input;
       lastTime = now;
   }
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void setup(){
    pinMode(pwm, OUTPUT);
    pinMode(hallsensor, INPUT);
    Serial.begin(9600);
    attachInterrupt(0, rpm, RISING); 
    SetTunings(20,0,0);
}

void loop(){
  NbTopsFan = 0;

  sei();
  delay (1000);
  cli();
  
  int leitura = analogRead(sensor_pin);
  float temperatura = 0.7363 * leitura - 13.7691;
  Serial.print("V :  ");
  Serial.println(leitura);
  Serial.print("C° : ");
  Serial.println(temperatura);

  valor = temperatura;

  if (Serial.available() > 0){
    String recebido = leStringSerial();
    valor = recebido.toFloat();
    Serial.println(valor);
  }

  Input = valor;

  Compute();

  Serial.print("error:");
  Serial.println(error);
  
  Serial.print("kp * error :");
  Serial.println(kp * error);
  
  Serial.print("ki * errSum :");
  Serial.println(ki * errSum);
  
  Serial.print("kd * errSum :");
  Serial.println(kd * dErr);
  
  Serial.print("Output:");
  Serial.println(Output);
  
  analogWrite(pwm,Output);
  
  Calc = ((NbTopsFan * 60)/2);
  Serial.print (Calc, DEC);
  Serial.print (" rpm\r\n"); 
}
