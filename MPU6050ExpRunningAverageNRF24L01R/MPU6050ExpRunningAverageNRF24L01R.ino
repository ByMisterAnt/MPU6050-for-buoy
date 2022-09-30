#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <GyverBME280.h> // подключаем библиотеку для датчика температуры и давления BMP280
#include <DHT.h>      // подключаем библиотеку для датчика температуры и влажности DHT11 (берем только влажность)

#define pin_dht 4

GyverBME280 bmp;
DHT dht(pin_dht, DHT11);  // сообщаем на каком порту будет датчик

RF24 radio(9, 10);  // "создать" модуль на пинах 9 и 10 Для Уно
//RF24 radio(9,53); // для Меги

byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
float tw=0,hw=0; // температура воды, высота волны
float ta=0, pa=0, rhoa=0; // температура, давление, влажность воздуха
float data[2];

void setup() {
  Serial.begin(9600);         // открываем порт для связи с ПК
  
  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);   // размер пакета, в байтах
  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.setChannel(0x60);     // выбираем канал (в котором нет шумов!)
  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  radio.powerUp();        // начать работу
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль

  //bmp.begin(0x76);        // запускаем датчик BMP280
  dht.begin();            // запускаем датчик DHT11
}

void loop() {
  while (radio.available(&pipeNo)) {        // слушаем эфир со всех труб
    radio.read(&data, sizeof(data));  // чиатем входящий сигнал
    tw=data[0]; // температура воды
    hw=2*data[1]; // высота волны
    //ta=bmp.readTemperature(); // температура воздуха с BMP280, С 
    //pa=bmp.readPressure()*0.007501; // атмосферное давление с BMP280, мм рт.ст.
    ta = dht.readTemperature();
    rhoa = dht.readHumidity(); // влажность воздуха с DHT11, %
    String rdata = String(rhoa,1)+" "+String(ta,1)+" "+String(tw,1)+" "+String(hw,1);
    Serial.println(rdata);
  }
}
