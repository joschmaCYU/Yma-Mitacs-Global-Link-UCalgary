#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>

SPIClass SPI_1(PA7, PA6, PA5);
SPIClass SPI_2(PB15, PB14, PB13);
SPIClass SPI_3(PC12, PC11, PC10);

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup() {
    SPI_1.begin();
    SPI_2.begin();
    SPI_3.begin();

    // Configuration stricte
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(chatter);
}

void loop() {
    str_msg.data = "Test de communication Nucleo OK !";
    chatter.publish(&str_msg);
    
    nh.spinOnce();
    delay(50); // Délai vital pour laisser respirer le port USB
}