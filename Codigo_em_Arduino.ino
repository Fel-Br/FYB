#include <PID_v1.h>

#define ENC_1 20
#define ENC_2 21
#define ENC_3 2
#define ENC_4 3

// Definição das variáveis utilizadas para receber comandos pelo Serial Monitor
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; 
char calibrate[]= "C";
char execute[] = "G";
char motor_calibration[] = "M";
char input_action[numChars] = {0};

float input_x = 0.0;
float input_y = 0.0;
float input_phi = 0.0;

static boolean recvInProgress = false;
static byte ndx = 0;
char startMarker = '<';
char endMarker = '>';
char rc;

boolean newData = false;

// Variaveis de projeto
float diameter = 25.0; //em mm

// Definição das variáveis utilizadas pelo motor

// Graus
float dg_1 = 0.0;
float dg_2 = 0.0;
float dg_3 = 0.0;
float dg_4 = 0.0;

// Posição

float pos_1 = 0.0;
float pos_2 = 0.0;
float pos_3 = 0.0;
float pos_4 = 0.0;

// Pulsos requeridos pelo motor

int pulse_req_1 = 0;
int pulse_req_2 = 0;
int pulse_req_3 = 0;
int pulse_req_4 = 0;

//Armazenamento de pulsos 
int store_pulse_req_1 = 0;
int store_pulse_req_2 = 0;
int store_pulse_req_3 = 0;
int store_pulse_req_4 = 0;

//Contagem de pulsos enviados pelo motor
volatile int encoderCount_1 = 0;
volatile int encoderCount_2 = 0;
volatile int encoderCount_3 = 0;
volatile int encoderCount_4 = 0;

//Tamanho dos cabos considerando o EE ao centro da área de trabalho
float len_current_1 = 214.73;
float len_current_2 = 214.73;
float len_current_3 = 214.73;
float len_current_4 = 214.73;

//Armazenamento do tamanho dos cabos
float len_current_old_1 = 0.0;
float len_current_old_2 = 0.0;
float len_current_old_3 = 0.0;
float len_current_old_4 = 0.0;

//Diferença do tamanho do cabo atual para o futuro real (Calculado baseado na quantidade de pulsos enviada)
float Delta_len_real_1 = 0.0;
float Delta_len_real_2 = 0.0;
float Delta_len_real_3 = 0.0;
float Delta_len_real_4 = 0.0;

//Tamanho real do cabo (Calculado baseado na quantidade de pulsos enviada)
float len_real_1 = 0.0;
float len_real_2 = 0.0;
float len_real_3 = 0.0;
float len_real_4 = 0.0;

//Objetivo de tamanho de cabo
float len_goal_1 = 0.0;
float len_goal_2 = 0.0;
float len_goal_3 = 0.0;
float len_goal_4 = 0.0;

//Difernça do tamanho do cabo atual para o futuro
float Delta_len_1 = 0.0;
float Delta_len_2 = 0.0;
float Delta_len_3 = 0.0;
float Delta_len_4 = 0.0;

//Centro do sistema
float x = 161.5;
float y = 175.4;
float phi = 0.0;

//Objetivo de centro do sistema
float x_goal = 161.5;
float y_goal = 175.4;
float phi_goal = 0.0;

//Variáveis para calculo de cinemática inversa, baseadas em calculos vetoriais
float Ax_1 = 0.0;
float Ax_2 = 0.0;
float Ax_3 = 320.28;
float Ax_4 = 320.28;

float Ay_1 = 0.0;
float Ay_2 = 348.28;
float Ay_3 = 0.0;
float Ay_4 = 348.28;

float Qx_1 = -16.11;
float Qx_2 = -16.11;
float Qx_3 = 16.11;
float Qx_4 = 16.11;

float Qy_1 = -26.00;
float Qy_2 = 26.00;
float Qy_3 = -26.00;
float Qy_4 = 26.00;

//Pulsos por rotação e graus por pulsos por rotação
float PPR_1 = 1525;
float PPR_2 = 1347;
float PPR_3 = 1549;
float PPR_4 = 1484;; 
float DPPR_1 = PPR_1/(2*PI);
float DPPR_2 = PPR_2/(2*PI);
float DPPR_3 = PPR_3/(2*PI);
float DPPR_4 = PPR_4/(2*PI);

//Diferença do sinal enviado para o sinal realizado pelo motor
float dif_signal_1 = 0.0;
float dif_signal_2 = 0.0;
float dif_signal_3 = 0.0;
float dif_signal_4 = 0.0;

float new_dif_signal_1 = 0.0;
float new_dif_signal_2 = 0.0;
float new_dif_signal_3 = 0.0;
float new_dif_signal_4 = 0.0;

//Variável de validação do motor, verifica se o motor já rodou a quantidade de pulsos necessário
int validation_motor_1 = 0;
int validation_motor_2 = 0;
int validation_motor_3 = 0;
int validation_motor_4 = 0;

//Variável que descreve a direção da rotação do motor. 1 libera cabo, -1 recolhe cabo
int direction_motor_1 = 0;
int direction_motor_2 = 0;
int direction_motor_3 = 0;
int direction_motor_4 = 0;


int setup_run = 0;

//Variável de controle. Reduz de pulsos requeridos ao enviar ao motor
float control_P_value_1 = 221;
float control_P_value_2 = 194;
float control_P_value_3 = 219;
float control_P_value_4 = 184;

// Função para o calculo de pulsos requeridos
float calc_pulse_req(float L, float PPR, float diameter) {
  float pulse_req = L*PPR/(PI*diameter);
  return pulse_req;
}
// Função para o calculo de delta L
float calc_delta_L(float L_i, float L_f) {
  float delta_L = (L_f - L_i);
  return delta_L;
}
// Função para o calculo do tamanho do cabo em x 
float calc_len_x(float calc_x, float calc_phi, float Qx, float Qy, float Ax){
  float len_x = calc_x + cos(phi)*Qx - sin(phi)*Qy - Ax;
  return len_x;
}
// Função para o calculo do tamanho do cabo em y
float calc_len_y(float calc_y,  float calc_phi, float Qx, float Qy, float Ay){
  float len_y = calc_y + sin(phi)*Qx + cos(phi)*Qy - Ay;
  return len_y;
}

float calc_len_run(float encoder_count, float diameter, float PPR){
  float len_run = encoder_count*PI*diameter/PPR;
  return len_run;
}

//Utiliza programação orientada a objeto para definir os comandos enviado ao motor
class DCMotor {  
  int spd = 255, pin1, pin2;
  
  public:  
  
    void Pinout(int in1, int in2){ // Pinout é o método para a declaração dos pinos que vão controlar o objeto motor
      pin1 = in1;
      pin2 = in2;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      }   
    void Speed(int in1){ // Speed é o método que irá ser responsável por salvar a velocidade de atuação do motor
      spd = in1;
      }     
    void Forward(){ // Forward é o método para fazer o motor girar para frente
      analogWrite(pin1, spd);
      digitalWrite(pin2, LOW);
      }   
    void Backward(){ // Backward é o método para fazer o motor girar para trás
      digitalWrite(pin1, LOW);
      analogWrite(pin2, spd);
      }
    void Stop(){ // Stop é o metodo para fazer o motor ficar parado.
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);

      }
   };
   DCMotor Motor1, Motor2, Motor3, Motor4; // Criação de dois objetos motores, já que usaremos dois motores, e eles já estão prontos para receber os comandos já configurados acima. 
void setup() {
  Serial.begin(19200);
  

  //Definição de pinos do motor
  Motor1.Pinout(11,12);
  Motor2.Pinout(9,10); 
  Motor3.Pinout(42,47); 
  Motor4.Pinout(7,8); 

  //Definição de velocidade dos motores. Velocidades baixas causam o sistema a não funcionar corretamente. Aconselhado <200
  Motor1.Speed(255);
  Motor2.Speed(255);
  Motor3.Speed(255);
  Motor4.Speed(255);
  //Função para receber pulsos enviados pelos encoders. Utilização a função CHANGE, ou seja, sempre que há alguma mudança no sinal, ele registra. Está conectado a apenas um dos sinais do encoder do motor.
  attachInterrupt(digitalPinToInterrupt(ENC_1), pulseInterrupt_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_2), pulseInterrupt_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_3), pulseInterrupt_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_4), pulseInterrupt_4, CHANGE);
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
        
        strcpy(tempChars, receivedChars);
        parseData();
        if(strstr(input_action,motor_calibration)>0){
          float direction = input_x; // 1 para liberar cabo, -1 para recolher cabo;
          float c_motor = input_y; // Qual motor mover;
          float time_count = input_phi; //Tempo estimado para motor ficar rodando, em milisegundos;
          if(c_motor == 1){
            if(direction == 1){
              Motor1.Forward();
              delay(time_count);
              Motor1.Stop();
            }
            if(direction == -1){
              Motor1.Backward();
              delay(time_count);
              Motor1.Stop();
            }
          }
          if(c_motor == 2){
            if(direction == 1){
              Motor2.Forward();
              delay(time_count);
              Motor2.Stop();
            }
            if(direction == -1){
              Motor2.Backward();
              delay(time_count);
              Motor2.Stop();
            }
          }
          if(c_motor == 3){
            if(direction == 1){
              Motor3.Forward();
              delay(time_count);
              Motor3.Stop();
            }
            if(direction == -1){
              Motor3.Backward();
              delay(time_count);
              Motor3.Stop();
            }
          }
          if(c_motor == 4){
            if(direction == 1){
              Motor4.Forward();
              delay(time_count);
              Motor4.Stop();
            }
            if(direction == -1){
              Motor4.Backward();
              delay(time_count);
              Motor4.Stop();
            }
          }
        
        
        }
        if(strstr(input_action,execute)> 0 || (strstr(input_action,calibrate) > 0) ){
          x_goal = input_x;
          y_goal = input_y;
          phi_goal = input_phi*71/4068.0;

        }
        newData = false;
  }
  //Quando os motores rodarem a quantidade necessárias de pulsos, a variável validation_motor_n vira 1. Se todas estiverem com valor 1, ele finaliza o ciclo e diz que a posição atual é a posição objetivo inserida.
  if (validation_motor_1 == 1 && validation_motor_2 == 1 && validation_motor_3 == 1 && validation_motor_4 == 1){

    if(strstr(input_action,execute) > 0){
      x = x_goal;
      y = y_goal;
      phi = phi_goal;
    }
    
    if(strstr(input_action,calibrate) > 0){
      x = 160.14;
      y = 174.14;
    }
    
    delay(3000);//Delay é necessário para dar tempo do motor parar e você poder ver exatamente quanto o encoder rodou. Aconselhável <1000. 3000 é com fator de segurnaça.
    dif_signal_1 = encoderCount_1 - pulse_req_1;
    Delta_len_real_1 = calc_len_run(encoderCount_1, diameter, PPR_1);
    len_real_1 = Delta_len_real_1*direction_motor_1;

    dif_signal_2 = encoderCount_2 - pulse_req_2;
    Delta_len_real_2 = calc_len_run(encoderCount_2, diameter, PPR_2);
    len_real_2 = Delta_len_real_2*direction_motor_2;//

    dif_signal_3 = encoderCount_3 - pulse_req_3;
    Delta_len_real_3 = calc_len_run(encoderCount_3, diameter, PPR_3);
    len_real_3 = Delta_len_real_3*direction_motor_3;

    dif_signal_4 = encoderCount_4 - pulse_req_4;
    Delta_len_real_4 = calc_len_run(encoderCount_4, diameter, PPR_4);
    len_real_4 = Delta_len_real_4*direction_motor_4;

    
    
    
    Serial.print("Pulse_req_1 :");
    Serial.println(store_pulse_req_1);
    Serial.print("pulse_req_2:");
    Serial.println(store_pulse_req_2);
    Serial.print("pulse_req_3:");
    Serial.println(store_pulse_req_3);
    Serial.print("pulse_req_4:");
    Serial.println(store_pulse_req_4);

    Serial.print("Enc_1:");
    Serial.println(encoderCount_1);
    Serial.print("Enc_2:");
    Serial.println(encoderCount_2);
    Serial.print("Enc_3:");
    Serial.println(encoderCount_3);
    Serial.print("Enc_4:");
    Serial.println(encoderCount_4);
    
    dif_signal_1 = encoderCount_1 - store_pulse_req_1;
    dif_signal_2 = encoderCount_2 - store_pulse_req_2;
    dif_signal_3 = encoderCount_3 - store_pulse_req_3;
    dif_signal_4 = encoderCount_4 - store_pulse_req_4;

    new_dif_signal_1 = dif_signal_1;
    new_dif_signal_2 = dif_signal_2;
    new_dif_signal_3 = dif_signal_3;
    new_dif_signal_4 = dif_signal_4;
    //Printa a diferença de sinais, entre solicitada e realmente entregue.
    Serial.print("dif_signal_1:");
    Serial.println(new_dif_signal_1);
    Serial.print("dif_signal_1:");
    Serial.println(new_dif_signal_2);
    Serial.print("dif_signal_3:");
    Serial.println(new_dif_signal_3);
    Serial.print("dif_signal_4:");
    Serial.println(new_dif_signal_4);

    //Zera as variáveis
    dif_signal_1 = 0;
    dif_signal_2 = 0;
    dif_signal_3 = 0;
    dif_signal_4 = 0;

    encoderCount_1 = 0.0;
    encoderCount_2 = 0.0;
    encoderCount_3 = 0.0;
    encoderCount_4 = 0.0;

    validation_motor_1 = 0;
    validation_motor_2 = 0;
    validation_motor_3 = 0;
    validation_motor_4 = 0;



  }
  if ((x != x_goal) or (y != y_goal) or (phi != phi_goal)){
    
   
    //Realiza os calculos para tamanho de cabo objetivo, diferneça comparando com o atual e a quantidade de pulsos requisitados que serão enviadas ao motor
    len_goal_1 = sqrt(pow(calc_len_x(x_goal, phi_goal, Qx_1, Qy_1, Ax_1),2) + pow(calc_len_y(y_goal, phi_goal, Qx_1, Qy_1, Ay_1),2));
    len_goal_2 = sqrt(pow(calc_len_x(x_goal, phi_goal, Qx_2, Qy_2, Ax_2),2) + pow(calc_len_y(y_goal, phi_goal, Qx_2, Qy_2, Ay_2),2));
    len_goal_3 = sqrt(pow(calc_len_x(x_goal, phi_goal, Qx_3, Qy_3, Ax_3),2) + pow(calc_len_y(y_goal, phi_goal, Qx_3, Qy_3, Ay_3),2));
    len_goal_4 = sqrt(pow(calc_len_x(x_goal, phi_goal, Qx_4, Qy_4, Ax_4),2) + pow(calc_len_y(y_goal, phi_goal, Qx_4, Qy_4, Ay_4),2));
    
    Delta_len_1 = calc_delta_L(len_current_1, len_goal_1);
    Delta_len_2 = calc_delta_L(len_current_2, len_goal_2);
    Delta_len_3 = calc_delta_L(len_current_3, len_goal_3);
    Delta_len_4 = calc_delta_L(len_current_4, len_goal_4);

    pulse_req_1 = calc_pulse_req(Delta_len_1,PPR_1, diameter);
    pulse_req_2 = calc_pulse_req(Delta_len_2,PPR_2, diameter);
    pulse_req_3 = calc_pulse_req(Delta_len_3,PPR_3, diameter);
    pulse_req_4 = calc_pulse_req(Delta_len_4,PPR_4, diameter);

    //LOOP PARA MOTOR 1
    //Se Pulse_req > 0, significia que é necessário liberar cabo.
    if(pulse_req_1 > 0){
      Motor1.Forward();
      direction_motor_1 = 1;
      pulse_req_1 = pulse_req_1 + new_dif_signal_1;
      if(encoderCount_1 > (pulse_req_1 - control_P_value_1 )){
        Serial.print("Pulse Req 1: ");
        Serial.println(pulse_req_1);
        Serial.print("delta len_1: ");
        Serial.println(Delta_len_1);
        Motor1.Stop();
        len_current_old_1 = len_current_1;
        len_current_1 = len_goal_1;
        validation_motor_1 = 1;
        store_pulse_req_1 = pulse_req_1;
      }
    }
    //Se Pulse_req > 0, significia que é necessário recolher.
    if(pulse_req_1 < 0){
        pulse_req_1 = pulse_req_1*(-1);
        Motor1.Backward();
        direction_motor_1 = -1;
        pulse_req_1 = pulse_req_1 + new_dif_signal_1; 
        if(encoderCount_1 > (pulse_req_1 - control_P_value_1 )){
          Serial.print("Pulse Req 1: ");
          Serial.println(pulse_req_1);
          Serial.print("delta len_1: ");
          Serial.println(Delta_len_1);
          Motor1.Stop();
          len_current_old_1 = len_current_1;
          len_current_1 = len_goal_1;
          validation_motor_1 = 1;
          store_pulse_req_1 = pulse_req_1;
      }
    }
    //LOOP PARA MOTOR 2
    if(pulse_req_2 > 0){
      Motor2.Forward();

      direction_motor_2 = 1;
      pulse_req_2 = pulse_req_2 + new_dif_signal_2;
      if(encoderCount_2 > (pulse_req_2 - control_P_value_2)){
        Serial.print("Pulse Req 2: ");
        Serial.println(pulse_req_2);
        Serial.print("delta len_2: ");
        Serial.println(Delta_len_2);
        Motor2.Stop();

        len_current_old_2 = len_current_2;
        len_current_2 = len_goal_2;
        validation_motor_2 = 1;
        store_pulse_req_2 = pulse_req_2;
      }
    }
    if(pulse_req_2 < 0){
        pulse_req_2 = pulse_req_2*(-1);
        Motor2.Backward();
        pulse_req_2 = pulse_req_2 + new_dif_signal_2;
        direction_motor_2 = -1;
        if(encoderCount_2 > (pulse_req_2 - control_P_value_2 )){
          Serial.print("Pulse Req 2: ");
          Serial.println(pulse_req_2);
          Serial.print("delta len_2: ");
          Serial.println(Delta_len_2);
          Motor2.Stop();

          len_current_old_2 = len_current_2;
          len_current_2 = len_goal_2;
          validation_motor_2 = 1;
          store_pulse_req_2 = pulse_req_2;
      }
    }
    //LOOP PARA MOTOR 3
    if(pulse_req_3 > 0){

      pulse_req_3 = pulse_req_3 + new_dif_signal_3;
      Motor3.Forward();
      direction_motor_3 = 1;
      if(encoderCount_3 > (pulse_req_3 - control_P_value_3)){
        Serial.print("Pulse Req 3: ");
        Serial.println(pulse_req_3);
        Serial.print("delta len_3: ");
        Serial.println(Delta_len_3);
        Motor3.Stop();
        len_current_old_3 = len_current_3;
        len_current_3 = len_goal_3;
        validation_motor_3 = 1;
        store_pulse_req_3 = pulse_req_3;
      }
    }
    if(pulse_req_3 < 0){
        pulse_req_3 = pulse_req_3*(-1);
        pulse_req_3 = pulse_req_3 + new_dif_signal_3;
        Motor3.Backward();
        direction_motor_3 = -1;
        if(encoderCount_3 > (pulse_req_3 - control_P_value_3)){
          Serial.print("Pulse Req 3: ");
          Serial.println(pulse_req_3);
          Serial.print("delta len_3: ");
          Serial.println(Delta_len_3);
          Motor3.Stop();
          len_current_old_3 = len_current_3;
          len_current_3 = len_goal_3;
          validation_motor_3 = 1;
          store_pulse_req_3 = pulse_req_3;
      }
    }
    //LOOP PARA MOTOR 4
    if(pulse_req_4 > 0){

      pulse_req_4 = pulse_req_4 + new_dif_signal_4;
      Motor4.Forward();
      direction_motor_4 = 1;
      if(encoderCount_4 > (pulse_req_4 - control_P_value_4)){
        Serial.print("Pulse Req 4: ");
        Serial.println(pulse_req_4);
        Serial.print("delta len_4: ");
        Serial.println(Delta_len_4);
        Motor4.Stop();
        len_current_old_4 = len_current_4;
        len_current_4 = len_goal_4;
        validation_motor_4 = 1;
        store_pulse_req_4 = pulse_req_4;
      }
    }
    if(pulse_req_4 < 0){
        pulse_req_4 = pulse_req_4*(-1);
        pulse_req_4 = pulse_req_4 + new_dif_signal_4;
        Motor4.Backward();
        direction_motor_4  = -1;
        if(encoderCount_4 > (pulse_req_4 - control_P_value_4)){
          Serial.print("Pulse Req 4:");
          Serial.println(pulse_req_4);
          Serial.print("delta len_4: ");
          Serial.println(Delta_len_4);
          Motor4.Stop();
          len_current_old_4 = len_current_4;
          len_current_4 = len_goal_4;
          validation_motor_4 = 1;
          store_pulse_req_4 = pulse_req_4;
      }
    }

    

  }
    
} 

//As funções em conjunto com AttachInterrupt somam sempre que há mudanças no pulso pelo encoder.
void pulseInterrupt_1() {
  encoderCount_1++;
}
void pulseInterrupt_2() {
  encoderCount_2++;
}
void pulseInterrupt_3() {
  encoderCount_3++;
}
void pulseInterrupt_4() {
  encoderCount_4++;
}

void readEncoder_1(){
  float b_1 = digitalRead(ENC_1);
  if(b_1>0){
    pos_1++;
  }
  else{
    pos_1--;
  }
}
void readEncoder_2(){
  float b_2 = digitalRead(ENC_2);
  if(b_2>0){
    pos_2++;
  }
  else{
    pos_2--;
  }
}
void readEncoder_3(){
  float b_3 = digitalRead(ENC_3);
  if(b_3>0){
    pos_3++;
  }
  else{
    pos_3--;
  }
}
void readEncoder_4(){
  float b_4 = digitalRead(ENC_4);
  if(b_4>0){
    pos_4++;
  }
  else{
    pos_4--;
  }
}

//============
//Utilizado para receber comandos pelo AttachInterrupt
void recvWithStartEndMarkers() {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
void parseData() {     
    char * strtokIndx; 
    strtokIndx = strtok(tempChars,",");      
    strcpy(input_action,strtokIndx);

    strtokIndx = strtok(NULL,",");      
    input_x = atof(strtokIndx); 
 
    strtokIndx = strtok(NULL, ","); 
    input_y = atof(strtokIndx);     

    strtokIndx = strtok(NULL, ",");
    input_phi = atof(strtokIndx);    

}
