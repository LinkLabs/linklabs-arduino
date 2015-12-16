#include "SymphonyLink.h"

#define SL_RESET_PIN 2
#define SL_BOOT_PIN 8
#define SL_IRQ_PIN 13


void setup() {
  //configure these to match your network and application tokens
  uint32_t network_token = 0x45156eb1;
  uint8_t app_token[APP_TOKEN_LEN] = {0xe2,0x71,0xa8,0xb1,0x74,0x01,0xb1,0xc5,0x15,0x40};

  ll_downlink_mode downlink_mode = LL_DL_ALWAYS_ON;
  uint8_t QOS = 15;
  uint32_t flags;

  //setup pin IO for standard linklabs eval board on an arduino.
  pinMode(SL_RESET_PIN, OUTPUT);
  pinMode(SL_BOOT_PIN, OUTPUT);
  pinMode(SL_IRQ_PIN, INPUT);

  digitalWrite(SL_BOOT_PIN, LOW);
  digitalWrite(SL_RESET_PIN, LOW);
  
  //setup serial port for module speed. default 115200.
  Serial1.begin(115200);

  //reset the module and wait 2 seconds for boot.
  digitalWrite(SL_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(SL_RESET_PIN, LOW);
  
  delay(2000);

  //send configuration data
  ll_config_set(network_token, app_token, downlink_mode, QOS);
  
  //wait until network connection established.
  while((flags&IRQ_FLAGS_CONNECTED)==0)
  {
    ll_irq_flags(0,&flags);
    delay(10);
  }

  //clear flags from reseting and connecting
  ll_irq_flags(IRQ_FLAGS_RESET|IRQ_FLAGS_CONNECTED,&flags);
}




void loop() {

  uint8_t val8;
  uint16_t val16;
  uint8_t data[1] = {0};
  uint32_t flags;
  
  //map 12bit analog value to 8bit value
  val16 = analogRead(A0);
  val8 = map(val16, 0, 1023, 0, 255);
  data[0] = val8;

  //clear the last transmit flag
  ll_irq_flags(IRQ_FLAGS_TX_DONE,&flags);
  delay(5);
  
  //send data
  ll_packet_send_ack(data,1);
  
  //wait 10 seconds before looping
  delay(10 * 1000);
}
