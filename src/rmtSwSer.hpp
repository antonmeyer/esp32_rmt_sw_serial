#include <stdint.h>

extern "C" {
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
}

typedef struct
{ // a half rmt item
  union
  {
    struct
    {
      uint16_t dur : 15;
      uint16_t lvl : 1;
    };
    uint16_t val;
  };
} rmt_pulse_t;

class ESP32rmtSwSer
{
public:
  // first constructors, then default constructor, then destructor
  //ESP32rmtSwSer(uint8_t rxchan, uint8_t rxpin);

  uint8_t ba[1024]; //ba is the bytearray for the result

  ESP32rmtSwSer();
  ~ESP32rmtSwSer(){};

  void setInit(uint8_t rxchan, uint8_t rxpin);

  void begin()
  {
    rmt_rx_start(rmt_rx.channel, true);
  };
  void stop()
  {
    rmt_rx_stop(rmt_rx.channel);
  };

  void hello();
  uint16_t getData();

private:
#define RMT_CLK_DIV (80) /*!< RMT counter clock divider  from 80 MHz ADB-> 1 MHz*/
  //#define RMT_TICK_1_US (2400000000 / RMT_CLK_DIV / 1000000) /*!< RMT counter value for 1 us.(Source clock is APB clock) */

#define P9600ticks (104)

  rmt_config_t rmt_rx;
  RingbufHandle_t rb = NULL;

  uint32_t bits = 0; // field to hold the bits of a byte
  uint8_t bitcnt = 0;
  uint8_t bitpos = 0; // counts bits of a symbol

  uint16_t pulse2data(rmt_pulse_t *pubuf, uint32_t buflen); 
};

static const char *TAG = "rmtSWSER";

//static const char *TAG = "rmtSWSER";

//ESP32rmtSwSer::ESP32rmtSwSer(uint8_t rxchan, uint8_t rxpin) {};
ESP32rmtSwSer::ESP32rmtSwSer(){};

void ESP32rmtSwSer::setInit(uint8_t rxchan, uint8_t rxpin)
{

  // Configure RMT
  rmt_rx.channel = (rmt_channel_t)rxchan;     // RMT channel
  rmt_rx.gpio_num = (gpio_num_t)rxpin;        // GPIO pin
  rmt_rx.clk_div = RMT_CLK_DIV;               // Clock divider
  rmt_rx.mem_block_num = 1;                   // number of mem blocks used
  rmt_rx.rmt_mode = RMT_MODE_RX;              // Receive mode
  rmt_rx.rx_config.filter_en = true;          // Enable filter
  rmt_rx.rx_config.filter_ticks_thresh = 255; // Filter all shorter then ticks source clock, not devided
  rmt_rx.rx_config.idle_threshold = 2000;     // Timeout after 2000 ticks

  rmt_config(&rmt_rx);                         // Init channel
  rmt_driver_install(rmt_rx.channel, 4000, 0); // Install driver with ring buffer size 4000
  //Serial.println("setup done");

  //get RMT RX ringbuffer rb
  rmt_get_ringbuf_handle((rmt_channel_t)rmt_rx.channel, &rb);
  ESP_LOGI(TAG, "init rmt channel: %d, pin: %d", rmt_rx.channel, rmt_rx.gpio_num);

  //should we start just here?
}

uint16_t ESP32rmtSwSer::getData()
{
  rmt_pulse_t *pulses;
  size_t rx_size = 0;
  uint8_t chunksize=0;

  //try to receive data from ringbuffer.
  //RMT driver will push all the data it receives to its ringbuffer.
  //We just need to parse the value and return the spaces of ringbuffer.
  //attantion it will block until timeout
  pulses = (rmt_pulse_t *)xRingbufferReceive(rb, &rx_size, 100);

  if (rx_size)
  {
    chunksize = pulse2data(pulses, rx_size / sizeof(rmt_pulse_t));

    vRingbufferReturnItem(rb, (void *)pulses);
    ESP_LOGD(TAG, "next chunk ch:%d size:%d bits: %X bitcnt: %X bitpos: %X\n", rmt_rx.channel, chunksize, bits, bitcnt, bitpos);

  };
  // else
  // ESP_LOGD(TAG, "rmtrx empty");

/*
  uint32_t status;
  rmt_get_status(rmt_rx.channel, &status);
  ESP_LOGV(TAG, "ch: %X status %X", rmt_rx.channel, status);
*/
  return chunksize;
}

uint16_t ESP32rmtSwSer::pulse2data(rmt_pulse_t *pubuf, uint32_t buflen)
{ //returns number of bytes in pulsebuffer

  /* the idea: devide the pulse length by the length of 1 bit (P9600ticks)
  that will result in the number of bits of the same value
  fill the byte with the bits
  make some checks
  first bit of a new byte needs to be a 0 otherwise we have not found start bit
    
  we can't finish a byte with 0 - stop bit would be missing
  with 1 we can only exact finish a byte as the next start bit would be a 0
  but we might run in a longer pause at high level (=1) so we limit longer high
  level and search in the next pulse for the start bit = 0

  We only need to add 1s to the bits as zero are already in.
*/
  if (bitpos > 12) //just in worst case
  {
    bitpos = 0;
    bitcnt = 0;
    bits = 0;
  }
  ESP_LOGD(TAG, "p2enter ch: %d bits: %X bitcnt: %X bitpos: %X", rmt_rx.channel, bits, bitcnt, bitpos);

  uint32_t bcnt =0;

  for (uint32_t i = 0; i < buflen; i++)
  {
    if (pubuf[i].lvl)
    { // 1s
      if (bitpos == 0)
      { // we miss the start bit = 0, lets search further
        ESP_LOGV(TAG, "ch: %d bits: %X bitcnt: %X bitpos: %X", rmt_rx.channel, bits, bitcnt, bitpos);
        continue;
      }

      if ((pubuf[i].dur == 0) && (bitpos > 0))
      { //special case at the end before a pause = endless 1

        bitcnt = 9; // all remaining bits to 1s, will be limited later
      }
      else
      {
        bitcnt = (pubuf[i].dur + (P9600ticks >> 1)) / P9600ticks; //duration of the pulse devided by length of 1 Bit, rounded
      }

      if ((bitpos + bitcnt) < 10)
      { // we are in the middle of the byte
        bits += (((1 << bitcnt) - 1) << (bitpos - 1));
        bitpos += bitcnt;
        ESP_LOGV(TAG, "ch: %d bits: %X bitcnt: %X bitpos: %X", rmt_rx.channel, bits, bitcnt, bitpos);
        continue; //continue with the next pulse of the same symbole
      }
      else
      { // we cut a longer high level and do not shift the stop bit in

        bits += (((1 << (9 - bitpos)) - 1) << (bitpos - 1));
        ESP_LOGD(TAG, "ch:%X bits:%02X", rmt_rx.channel, bits);
        //add bits to byte array
        ba[bcnt++]= bits;

        bits = 0;
        bitpos = 0;
        //continue with the next pulse and next symbol
        continue;
      }
    }
    else
    {
      //0 we do not need to change the bits field with 0, just checks and count up
      bitcnt = (pubuf[i].dur + (P9600ticks >> 1)) / P9600ticks; //round
      ESP_LOGD(TAG, " ch: %d bits: %X bitcnt: %d bitpos: %d dur:%d 0", rmt_rx.channel, bits, bitcnt, bitpos, pubuf[i].dur);
      bitpos += bitcnt;
      if (bitpos > 9)
      {
        ESP_LOGD(TAG, " ch: %d bits: %X bitcnt: %d bitpos: %d dur:%d 0", rmt_rx.channel, bits, bitcnt, bitpos, pubuf[i].dur);
        bitpos = 0;
        bits =0;
        bitcnt = 0;
        return bcnt; // we miss the stop bit flush the rest of the buffer
      }

      continue; // next pulse we can not finish a byte with a 0
    }
  }

  ESP_LOGV(TAG, "ch: %d bits: %X bitcnt: %X bitpos: %X", rmt_rx.channel, bits, bitcnt, bitpos);
  return bcnt;
}

