#include "I2Cdev.h"
#include "MPU6050.h"
#include "control.h"

#include <SPI.h>          // библиотека для работы с шиной SPI
#include "nRF24L01.h"     // библиотека радиомодуля
#include "RF24.h"         // ещё библиотека радиомодуля

#include <microDS18B20.h> // библиотека датчика температуры

#define TO_DEG 57.3 // 180/pi = 57.295779513082320876798154814105
#define T_OUT 10 // каждый 10 миллисекунд будем проводить вычисления 
#define KF 0.1 // коэффициент комплементарного фильтра

RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

MPU6050 accelgyro;
expRunningAverage Angle_ax,Angle_ay;
expRunningAverage Ax,Ay,Az;

MicroDS18B20 <2> sensor;  // датчик DS18B20 на D2

unsigned long timer1=0, timer2=0, timer3=0;
int16_t ax_raw, ay_raw, az_raw, ax_raw_g, ay_raw_g, az_raw_g, gx_raw, gy_raw, gz_raw;
float ax, ay, az, ax_g, ay_g, az_g, ax_n, ay_n, az_n, gx, gy, gz;
float angle_ax, angle_ay, angle_gx=0, angle_gy=0, angle_gz=0, angle_x=0, angle_y=0, angle_z=0;
float x_n, y_n, z_n; 
float prev_ax_n=0, prev_ay_n=0, prev_az_n=0;
unsigned long circle=0;
unsigned long kx=0, ky=0, kz=0;
float fx=0, fy=0, fz=0;
float az_n_max=0, az_n_max1=0;
float sum=0;
float tw=0, hw=0; // температура воды, высота волны
float data[2];

//float centering_ax = 0, noize_ax = 0;
//float centering_ay = 0, noize_ay = 0;
//float centering_az = 0, noize_az = 0;
//float centering_gx = 0, noize_gx = 0;
//float centering_gy = 0, noize_gy = 0;
//float centering_gz = 0, noize_gz = 0;

void setup() {
    Serial.begin(9600);
    //Serial.println("angle_ax,angle_gx,angle_x");
    accelgyro.initialize(); 
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // предел измерений акселерометра(2,4,8,16 g)
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // предел измерений гироскопа(250,500,1000,2000 град/с)
    
    // Экспоненциальное бегущее среднее
    Angle_ax.set_values(0.3, 0.0); // (коэффициент фильтрации(0.0-1.0), начальное значение)
    Angle_ay.set_values(0.3, 0.0);
    Ax.set_values(0.3, 0.0);
    Ay.set_values(0.3, 0.0);
    Az.set_values(0.3, 0.0);

    pinMode(4, OUTPUT); // Пин D4 на выход

    radio.begin();              // активировать модуль
    radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
    radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
    radio.setPayloadSize(32);   // размер пакета, в байтах
    radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
    radio.setChannel(0x60);             // выбираем канал (в котором нет шумов!)
    radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
    //должна быть одинакова на приёмнике и передатчике!
    //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

    radio.powerUp();        // начать работу
    radio.stopListening();  // не слушаем радиоэфир, мы передатчик

    sensor.setResolution(9); // устанавливаем разрешение датчика ds18b20 от 9 до 12 бит
    
//    for (int i = 0; i < 1000; i++)
//    {
//      accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
//
//      noize_ax = ax_raw;
//      noize_ay = ay_raw;
//      noize_az = az_raw;
//      centering_ax += noize_ax;
//      centering_ay += noize_ay;
//      centering_az += noize_az;
//
//      if (i >= 992)
//      {
//          //F1[i - 992] = noize_ax;
//          //F2[i - 992] = noize_ay;
//          //F3[i - 992] = noize_az;
//      }
//    }
//    
//    centering_ax = centering_ax / 1000;
//    centering_ay = centering_ay / 1000;
//    centering_az = centering_az / 1000;
//
//    for (int i = 0; i < 1000; i++)
//    {
//      accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
//
//      noize_gx = gx_raw;
//      noize_gy = gy_raw;
//      noize_gz = gz_raw;
//      centering_gx += noize_gx;
//      centering_gy += noize_gy;
//      centering_gz += noize_gz;
//
//      if (i >= 992)
//      {
//          //F1[i - 992] = noize_ax;
//          //F2[i - 992] = noize_ay;
//          //F3[i - 992] = noize_az;
//      }
//    }
//    
//    centering_gx = centering_gx / 1000;
//    centering_gy = centering_gy / 1000;
//    centering_gz = centering_gz / 1000;
}

void loop() {
    if( millis()-timer1 > T_OUT ){
        timer1 = millis(); // Обнулить таймер 
        
        ///////////////////////////////////// Считывание показаний с MPU6050 //////////////////////////////////
        accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        
        //////////////////////////////////////// Вычисление углов /////////////////////////////////////////////
        // ограничиваем +-1g
        ax_raw_g = constrain(ax_raw, -16384, 16384);
        ay_raw_g = constrain(ay_raw, -16384, 16384);
        az_raw_g = constrain(az_raw, -16384, 16384);

        // переводим в +-1.0
        ax_g = ax_raw_g / 16384.0;
        ay_g = ay_raw_g / 16384.0;
        az_g = az_raw_g / 16384.0;
        //Serial.print(ax_g);Serial.print("\t");
        //Serial.print(ay_g);Serial.print("\t");
        //Serial.print(az_g);Serial.print("\t");
           
        // угол наклона по акселерометру
        angle_ax = 90.0 - degrees(acos(ay_g));
        angle_ay = 90.0 - degrees(acos(ax_g));
        //Serial.print(angle_ax);Serial.print("\t");
        //Serial.print(angle_ay);Serial.print("\t");
        
        angle_ax = Angle_ax.expRunningAverage_solver(angle_ax);
        angle_ay = Angle_ay.expRunningAverage_solver(angle_ay);
        //Serial.print(angle_ax);Serial.print("\t");
        //Serial.print(angle_ay);Serial.println("\t");

        // преобразование сырых данных гироскопа в град/сек 250 град/сек = 32768/250
        gx = gx_raw / 32768.0 * 250;
        gy = gy_raw / 32768.0 * 250;
        gz = gz_raw / 32768.0 * 250;
        //Serial.print(gx);Serial.print("\t");
        //Serial.print(gy);Serial.print("\t");
        //Serial.print(gz);Serial.println("\t");
        
        // угол наклона по гироскопу
        angle_gx = angle_gx + gx * T_OUT/1000.0;
        angle_gy = angle_gy + gy * T_OUT/1000.0;
        angle_gz = angle_gz + gz * T_OUT/1000.0;
        //Serial.print(angle_gx);Serial.print("\t");
        //Serial.print(angle_gy);Serial.println("\t");

        // угол наклона по акселерометру и гироскопу
        angle_x = (1-KF)*(angle_x+gx*T_OUT/1000.0) + KF*angle_ax;
        angle_y = (1-KF)*(angle_y+gy*T_OUT/1000.0) + KF*angle_ay;
        angle_z = 0;
        //Serial.print(angle_x);Serial.println("\t");
        //Serial.print(angle_y);Serial.println("\t");      
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////// Вычисление ускорений ///////////////////////////////////////
        // преобразование сырых данных акселерометра в G 2G = 32768/2
        ax = ax_raw / 32768.0 * 2;
        ay = ay_raw / 32768.0 * 2;
        az = az_raw / 32768.0 * 2;
        
        ax = Ax.expRunningAverage_solver(ax);
        ay = Ay.expRunningAverage_solver(ay);
        az = Az.expRunningAverage_solver(az);
        //Serial.print(ax);Serial.print("\t");
        //Serial.print(ay);Serial.print("\t");
        //Serial.print(az);Serial.println("\t");
        
        // ускорения датчика в связанной СК (без учета ускорения свободного падения)
        ax=ax-1*sin((angle_y/TO_DEG));
        ay=ay-1*sin((angle_x/TO_DEG));
        az=az-1*cos(angle_y/TO_DEG)*cos(angle_x/TO_DEG);
        
        // преобразование ускорений из связанной СК в нормальную СК
        ax_n=(cos(angle_x/TO_DEG))*ax - (cos(angle_y/TO_DEG)*sin(angle_x/TO_DEG))*ay + (sin(angle_y/TO_DEG)*sin(angle_x/TO_DEG))*az;
        ay_n=(sin(angle_x/TO_DEG))*ax + (cos(angle_y/TO_DEG)*cos(angle_x/TO_DEG))*ay - (sin(angle_y/TO_DEG)*cos(angle_x/TO_DEG))*az;
        az_n=(0)*ax + (sin(angle_y/TO_DEG))*ay + (cos(angle_y/TO_DEG))*az;
        //Serial.print(ax_n);Serial.print("\t");
        //Serial.print(ay_n);Serial.print("\t");
        //Serial.print(az_n);Serial.println("\t");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////// Вычисление отклонения //////////////////////////////////////
        //arr1_az_n[circle1]=abs(az_n);
        if(abs(az_n)>az_n_max1) az_n_max1 = abs(az_n);
        
        if(prev_az_n*az_n<0){
          sum+=az_n_max1;
          az_n_max1=0;
          kz++;
        }

        if(circle>=89999){
          if(kz<2)hw=0;
          else{
          fz=kz/1800.0;
          az_n_max=sum/kz;
          z_n=(9.81*az_n_max)/(2*2*3.14*3.14*fz*fz);
          hw=2*z_n; //м
          }
          //Serial.print(fz);Serial.print("\t");
          //Serial.print(kz);Serial.print("\t");
          Serial.print(hw);Serial.print("\t");
          az_n_max1=0;
          circle=0;
          kz=0;
          sum=0;

          // запрашиваем новое измерение
          sensor.requestTemp();
          // читаем значение
          if (sensor.readTemp()) {
            tw = sensor.getTemp();
            Serial.println(tw);
          }
        }
        prev_az_n=az_n;
        circle++;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    ///////////////////////////////////////// Передача с помощью NRF24L01 /////////////////////////////////////
    if (millis()-timer2 > 900000) {
      data[0]=tw;
      data[1]=hw;
      radio.write(&data, sizeof(data));
    }
    if (millis()-timer2 > 1000000) {
      timer2 = millis(); // Обнулить таймер 
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////// Пробуждение аккумулятора ///////////////////////////////////////
    if (millis()-timer3 > 9000) {
      digitalWrite(4,1); // Подать 1 на пин D4 
    }      
    if (millis()-timer3 > 10000) {
      timer3 = millis(); // Обнулить таймер
      digitalWrite(4,0); // Подать 0 на пин D4
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
}
