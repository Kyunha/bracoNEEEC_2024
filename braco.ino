#include <Servo.h>

Servo servo_a, servo_b, servo_c, servo_d, servo_e; //declarar objetos dos servos
int angle_a=0; //declarar os angulos referentes a cada servo
unsigned long timer; //inicializar contador de tempo

void setup() {
    servo_a.attach(); //attach de cada servo ao respetivo pino
    servo_b.attach();
    servo_c.attach();
    servo_d.attach();
    servo_e.attach();
    pinMode(A0, INPUT);
    Serial.begin(9600); //iniciar comunicaçao serial (por usb)
    timer = millis(); //começar a contar tempo
}

void loop() {
    if ((millis() - timer) > 100) { //contar passagem do tempo
        Y_axis = analogRead(A0); //ler joystick
        if(Y_axis > 600){
            angle_a++;
            angle_a = angle_a>1023?1023:angle_a; //estabelecer limite superior no valor do angulo
            servo_a.write(angle_a);
        } else if( Y_axis < 400 ) {
            angle_a--;
            angle_a = angle_a<0?0:angle_a;  //estabelecer limite inferior no valor do angulo
            servo_a.write(angle_a);
        }
        timer=millis(); //reset ao timer
    }
}
