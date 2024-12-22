#include <Servo.h>


//VARIABLES

unsigned long timer; //inicializar contador de tempo

const float x0 = 0.0, y0 = 11.4; //P0 coords in cm
const float L1 = 13.1, L2 = 9.1, L3 = 13.5; //links lengths in cm
Servo servo_base, servo_shoulder, servo_elbow, servo_wrist, servo_claw; //declarar objetos dos servos

int X_axis, Y_axis; //eixos do joystick range(left/0:511/mid/512:1023/right)
bool click; //variável joystick click
unsigned long click_timer;  //inicializar contador de pausa para click

int mode = 0; //alterna input do joystick entre base e claw (0), orientação do end-effector (1) e coords cartesianas do end-effector (2)

int base = 90, claw = 180;  //ângulos das joints em graus (inteiros)
float cur_ang[3] = {90.0, 90.0, 90.0}, tar_ang[3]; //current e target angles vindos da IK {shoulder, elbow, wrist}
float cur_eff[3] = {x0, y0 + L1 + L2 + L3, 90.0}, tar_eff[3] = {x0, y0 + L1 + L2 + L3, 90.0}, temp_eff[3]; //current e target end-effector {x_eff, y_eff e orientação} para Inverse Kinematics e temporary end-effector para Forward Kinematics validar o target

float SIN, COS; //para o joystick no mode 1
float raio_trig; //para o joystick no mode 1

float distance; //para o joystick no mode 2

float theta = 90.0 , phi, alpha, sigma, delta, beta, gamma; //all float angles needed:
/*
  * theta é a orientação do end-effector (vértice P2, do semi-eixo horizontal à direita até ao end-effector); basicamente é a orientação de L3
  * phi é a posição angular do end-effector (P3) relativamente a P0 (vértice P0, do semi-eixo positivo Ox até ao end-effector)
  * alpha é o shoulder -> cur_ang[0]; alpha = sigma + delta
  * sigma é o ângulo (P2, P0, P1); se alpha = sigma + delta => configuração elbow up; se alpha = - sigma + delta => configuração elbow down
  * delta é a posição angular de P2 relativamente a P0 (vértice P0, do semi-eixo positivo Ox até P2)
  * beta é o elbow -> cur_ang[1]
  * gamma é o wrist -> cur_ang[2]
*/
float P2[2];  //coords P2 para IK em cm
float d;  //distância de P2 a P0 para IK em cm
bool elbow_config[2]; //flags para indicar se as configurações elbow up/down, respetivamente, são válidas ou não


//METHODS

/** @brief valida a tar_eff para garantir que é possível encontrar tar_ang válidos para a alcançar
  * @return true se tar_eff é validamente atingível; false caso não seja
  */
bool reachable(){
  //if()
  return true;
}

/** @brief calcula os tar_ang para o tar_eff
  */
void IK(){  //Inverse Kinematics
//calcular P2
  P2[0] = tar_eff[0] - L3 * cos(theta);
  P2[1] = tar_eff[1] - L3 * sin(theta);

//calcular distância de P2 a P0
  d = sqrt(sq(P2[0] - x0) + sq(P2[1] - y0));

//calcular delta como declive
  delta = atan2(P2[1] - y0, P2[0] - x0) * 180 / M_PI;

//calcular sigma em recurso à Lei dos Cossenos
  sigma = acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)) * 180 / M_PI;

//definir elbow_config
  elbow_config[0] = (sigma + delta) > 180 ? 0 : 1;  //configuração elbow up
  elbow_config[1] = (- sigma + delta) < 0 ? 0 : 1;  //configuração elbow down

//calcular alpha (preferencialmente, sempre elbow up)
  if(elbow_config[0]) alpha = sigma + delta;
  else if(elbow_config[1]) alpha = - sigma + delta;
  
//calcular beta em recurso à Lei dos Cossenos
  beta = asin((d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)) * 180 / M_PI;

//ajustar beta à configuração elbow up/down a ser utilizada
  beta = elbow_config[0] ? beta : 180 - beta;

//calcular gamma através da relação theta = alpha + beta - 180 + gamma
  gamma = theta + 180 - alpha - beta;

//atribuir os valores alpha, beta e gamma a tar_ang
  tar_ang[0] = alpha;
  tar_ang[1] = beta;
  tar_ang[2] = gamma;
}

/** @brief valida os tar_ang para alcançar o tar_eff (confirma a IK e verifica limites dos ângulos)
  * @return true se tar_ang sejam válidos; false caso não sejam
  */
bool FK(){  //Forward Kinematics
  temp_eff[0] = x0 + L1 * cos(tar_ang[0]) + L2 * cos(tar_ang[0] - 90 + tar_ang[1]) + L3 * cos(tar_ang[0] + tar_ang[1] - 180 + tar_ang[2]);
  temp_eff[1] = y0 + L1 * sin(tar_ang[0]) + L2 * sin(tar_ang[0] - 90 + tar_ang[1]) + L3 * sin(tar_ang[0] + tar_ang[1] - 180 + tar_ang[2]);
  temp_eff[2] = tar_ang[0] + tar_ang[1] - 180 + tar_ang[2];

  if((fabs(tar_eff[0] - temp_eff[0]) < 1) && (fabs(tar_eff[1] - temp_eff[1]) < 1) && (fabs(tar_eff[2] - temp_eff[2]) < 1)){  //confirmar IK
    if(tar_ang[0]){
      return true;
    }
  }
  return false;
}


void setup(){
//attach de cada servo ao respetivo pino
  servo_base.attach(5);
  servo_shoulder.attach(9);
  servo_elbow.attach(10);
  servo_wrist.attach(11);
  servo_claw.attach(6);

//inicialização dos servos
  servo_base.write(base);
  servo_shoulder.write(cur_ang[0]);
  servo_elbow.write(cur_ang[1]);
  servo_wrist.write(cur_ang[2]);
  servo_claw.write(claw);

//indicação joystick inputs
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(7, INPUT);

  Serial.begin(9600); //iniciar comunicaçao serial (por usb)

//começar a contar tempo
  timer = millis();
  click_timer = millis();
}


void loop(){
//alternar mode com click
  if((millis() - click_timer) > 10){
    click = digitalRead(7);
    if(click){
      click_timer = millis(); //reset click_timer
      mode ++;
      if(mode == 3) mode = 0;
    }
  }

//atuação em cada mode
  if((millis() - timer) > 100){ //contar passagem do tempo
    if(mode == 0){  //mode 0  //joystick atualiza base (X) e claw (Y)
    //ler joystick
      X_axis = analogRead(A0);
      Y_axis = analogRead(A1);
      
    //base
      if(X_axis > 612) base ++;
      else if(X_axis < 411) base --;
      base = constrain(base, 0, 180);
      servo_base.write(base);

    //claw
      if(Y_axis > 612) claw --;
      else if(Y_axis < 411) claw ++;
      claw = constrain(claw, 0, 180);
      servo_claw.write(claw);
    } //end mode 0

    else if(mode==1){ //mode 1  //joystick atualiza cur_eff[2] (end-effector orientation)
    //ler joystick
      X_axis = analogRead(A0);
      Y_axis = analogRead(A1);

    //centrar em 0 cada um dos eixos (range de [0, 1023] passa a ser [-512, 511]) e escalá-los para os valores apropriados (intervalo [-1, 1])
      COS = - (float)(X_axis - 512.0) / 512.0;
      SIN = (float)(Y_axis - 512.0) / 512.0;

    //aplicar deadzone
      COS = abs(COS) < 0.05 ? 0.00 : COS;
      SIN = abs(SIN) < 0.05 ? 0.00 : SIN;

    //normalizar o raio_trig
      //isto é feito pois o joystick consegue dar input de SIN = 1.00 & COS = 1.00, o que não deve acontecer num círculo trigonométrico
      raio_trig = sqrt(sq(COS) + sq(SIN));  //obter a diferença ("distância") do joystick à posição de equilíbrio (centro do joystick)
      COS /= raio_trig;
      SIN /= raio_trig;

    //calcular o ângulo theta e converter em graus
      if(sq(COS) + sq(SIN) == 1){
        if(COS == 0){
          if(SIN > 0){
            theta = 90.0;
          }
          else if(SIN < 0){
            theta = -90.0;
          }
        }
        else theta = atan2(SIN, COS) * 180 / M_PI; //atan2 tem já em consideração o quadrante do ângulo
      //temos o ângulo em graus e no intervalo [-179, 180]
      }
      tar_eff[2] = theta;
    } //end mode 1

    else{ //mode 2  //joystick atualiza tar_eff[0 & 1] (X para 0 & Y para 1)
    //ler joystick
      X_axis = analogRead(A0);
      Y_axis = analogRead(A1);

    //x_eff
      if(X_axis > 612) tar_eff[0] += 1;
      else if(X_axis < 411) tar_eff[0] -= 1;
      tar_eff[0] = sqrt(sq(tar_eff[0] - x0) + sq(tar_eff[1] - y0)) > L1 + L2 + L3 ? cur_eff[0] : tar_eff[0]; //estabelecer limite superior no valor de x_eff tendo em conta y_eff
      tar_eff[0] = sqrt(sq(tar_eff[0] - x0) + sq(tar_eff[1] - y0)) < sqrt(sq(L1 - L3) + sq(L2)) ? cur_eff[0] : tar_eff[0]; //estabelecer limite inferior no valor de x_eff tendo em conta y_eff

    //y_eff
      if(Y_axis > 612) tar_eff[1] += 1;
      else if(Y_axis < 411) tar_eff[1] = tar_eff[1] <= 1.3 ? 1.3 : tar_eff[1] -= 1; //estabelecer limite inferior absoluto no valor de y_eff
      tar_eff[1] = sqrt(sq(tar_eff[0] - x0) + sq(tar_eff[1] - y0)) > L1 + L2 + L3 ? cur_eff[1] : tar_eff[1]; //estabelecer limite superior no valor de y_eff tendo em conta x_eff
      tar_eff[1] = sqrt(sq(tar_eff[0] - x0) + sq(tar_eff[1] - y0)) < sqrt(sq(L1 - L3) + sq(L2)) ? cur_eff[1] : tar_eff[1] --;  //estabelecer limite inferior no valor de y_eff tendo em conta x_eff
//Serial.println(Y_axis);Serial.print("     ");Serial.print(tar_eff[1]);Serial.print("     ");Serial.print(sqrt(sq(tar_eff[0] - x0) + sq(tar_eff[1] - y0)));Serial.print("     ");Serial.print(sqrt(sq(L1 - L3) + sq(L2)));Serial.print("     ");Serial.println(L1 + L2 + L3);
    } //end mode 2

  //atualização do estado
    if((cur_eff[0] != tar_eff[0]) || (cur_eff[1] != tar_eff[1]) || (cur_eff[2] != tar_eff[2])){ //caso tenham havido alterações nos modes 1 ou 2
      //if(reachable()){
        IK(); //calcular e atualizar tar_ang de acordo com tar_eff
//        if(FK()){ //verificar tar_ang calculando temp_eff de acordo com tar_ang e comparando com tar_eff
        //atualizar cur_eff
          cur_eff[0] = tar_eff[0];
          cur_eff[1] = tar_eff[1];
          cur_eff[2] = tar_eff[2];

        //atualizar cur_ang
          cur_ang[0] = tar_ang[0];
          cur_ang[1] = tar_ang[1];
          cur_ang[2] = tar_ang[2];

        //attribuir cada um dos cur_ang aos servos respetivos
          servo_shoulder.write(cur_ang[0]);
          servo_elbow.write(cur_ang[1]);
          servo_wrist.write(cur_ang[2]);
          Serial.println(cur_ang[0]);Serial.print("     ");Serial.print(cur_ang[1]);Serial.print("     ");Serial.println(cur_ang[2]);//Serial.print("     ");Serial.print(sqrt(sq(L1 - L3) + sq(L2)));Serial.print("     ");Serial.println(L1 + L2 + L3);
//        }
      //}
    }
    
    timer = millis(); //reset ao timer
  }
}
