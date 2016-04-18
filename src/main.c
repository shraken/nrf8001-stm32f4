#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <usart.h>
#include <debug.h>
#include <spi.h>
#include <millis.h>
#include <globals.h>

#include <lib_aci.h>
#include <aci_setup.h>
#include "uart_over_ble.h"
#include <io_support.h>

#include "services.h"

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

#define RXBUFFERSIZE   20

extern uint8_t Rx_Flag_read;

/* Store the setup for the nRF8001 in the flash of the AVR to save on RAM */
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;


// aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;

/*
Temporary buffers for sending ACI commands
*/
static hal_aci_evt_t  aci_data;
//static hal_aci_data_t aci_cmd;

/*
Timing change state variable
*/
static bool timing_change_done          = false;

/*
Used to test the UART TX characteristic notification
*/
static uart_over_ble_t uart_over_ble;
uint8_t         uart_buffer[RXBUFFERSIZE];
static uint8_t         uart_buffer_len = 0;
static uint8_t         dummychar = 0;

bool stringComplete = false;  // whether the string is complete
uint8_t stringIndex = 0;      //Initialize the index to store incoming chars

/*
Description:

In this template we are using the BTLE as a UART and can send and receive packets.
The maximum size of a packet is 20 bytes.
When a command it received a response(s) are transmitted back.
Since the response is done using a Notification the peer must have opened it(subscribed to it) before any packet is transmitted.
The pipe for the UART_TX becomes available once the peer opens it.
See section 20.4.1 -> Opening a Transmit pipe
In the master control panel, clicking Enable Services will open all the pipes on the nRF8001.

The ACI Evt Data Credit provides the radio level ack of a transmitted packet.
*/

void nrf8001_setup()
{
  /**
  Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*)setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /*
  Tell the ACI library, the MCU to nRF8001 pin connections.
  The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = REQN_PIN; //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.rdyn_pin   = RDYN_PIN; //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.mosi_pin   = MOSI_PIN;
  aci_state.aci_pins.miso_pin   = MISO_PIN;
  aci_state.aci_pins.sck_pin    = SCLK_PIN;

  //aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed
  
  aci_state.aci_pins.reset_pin              = RESET_PIN; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number       = 1;

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);
}

void uart_over_ble_init(void)
{
  uart_over_ble.uart_rts_local = true;
}

bool uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1))
  {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status)
    {
      aci_state.data_credit_available--;
    }
  }

  return status;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    printf("%02X\r\n", *byte);

    switch(*byte)
    {
      /*
      Queues a ACI Disconnect to the nRF8001 when this packet is received.
      May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
        Parameters:
        None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        status = true;
        break;


      /*
      Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        /*
        Parameters:
        Connection interval min: 2 bytes
        Connection interval max: 2 bytes
        Slave latency:           2 bytes
        Timeout:                 2 bytes
        Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
        Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
        conn_params = (aci_ll_conn_params_t *)(byte+1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                                conn_params->max_conn_interval,
                                conn_params->slave_latency,
                                conn_params->timeout_mult);
        status = true;
        break;

      /*
      Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;


      /*
      Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }

  return status;
}


void aci_loop()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;
    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            printf("Evt Device Started: Setup");
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            printf("Evt Device Started: Standby");
            //Looking for an iPhone by sending radio advertisements
            //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Handle the HW error event correctly.
            }
            else
            {
              lib_aci_connect(0/* in seconds : 0 means forever */, 0x0050 /* advertising interval 50ms*/);
              printf("Advertising started : Tap Connect on the nRF UART app");
            }

            break;
        }
      }
      break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          printf("ACI Command %02X\r\n", aci_evt->params.cmd_rsp.cmd_opcode);
          printf("Evt Cmd respone: Status %02X\r\n", aci_evt->params.cmd_rsp.cmd_status);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        printf("Evt Connected");
        uart_over_ble_init();
        timing_change_done              = false;
        aci_state.data_credit_available = aci_state.data_credit_total;

        /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        printf("Evt Pipe Status");
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;

          char hello[]="Hello World, works";
          uart_tx((uint8_t *)&hello[0], strlen(hello));
          printf("Sending : %s", hello);
        }
        break;

      case ACI_EVT_TIMING:
        printf("Evt link connection interval changed");
        lib_aci_set_local_data(&aci_state,
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                                (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        printf("Evt Disconnected/Advertising timed out");
        lib_aci_connect(0/* in seconds  : 0 means forever */, 0x0050 /* advertising interval 50ms*/);
        printf("Advertising started. Tap Connect on the nRF UART app");
        break;

      case ACI_EVT_DATA_RECEIVED:
        printf("Pipe Number: ");
        printf("%d\r\n", aci_evt->params.data_received.rx_data.pipe_number);

        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
          {

            printf(" Data(Hex) : ");
            for(int i=0; i<aci_evt->len - 2; i++)
            {
              printf("%c", (char)aci_evt->params.data_received.rx_data.aci_data[i]);
              uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
              printf(" ");
            }
            uart_buffer_len = aci_evt->len - 2;
            printf("\r\n");
            if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
            {
              /*Do this to test the loopback otherwise comment it out*/
              /*
              if (!uart_tx(&uart_buffer[0], aci_evt->len - 2))
              {
                Serial.println(F("UART loopback failed"));
              }
              else
              {
                Serial.println(F("UART loopback OK"));
              }
              */
            }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        
        printf("ACI Evt Pipe Error: Pipe #: %d", aci_evt->params.pipe_error.pipe_number);
        printf("Pipe Error Code: 0x%02X", aci_evt->params.pipe_error.error_code);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        printf("HW error: %d\r\n", aci_evt->params.hw_error.line_num);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          printf("%c", (aci_evt->params.hw_error.file_name[counter]));
        }

        printf("\r\n");
        lib_aci_connect(0, 0x0050);
        printf("Advertising started. Tap Connect on the nRF UART app");
        break;

    }
  }
  else
  {
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}

void print_setup_messages()
{
    int i, k;

    for (i = 0; i < NB_SETUP_MESSAGES; i++) {
        printf("index: %02x\r\n", setup_msgs[i].status_byte);

        for (k = 0; k < HAL_ACI_MAX_LENGTH; k++) {
            printf("%02x:", setup_msgs[i].buffer[k]); 
        }
        printf("\r\n");
    }
}

int main(void) {
    static uint32_t main_count = 0;
    static uint32_t rx_count = 0;
    char buffer[MAX_STR_LEN];

    init_usart2(115200); // initialize the USART peripheral
    millis_init();

    print_setup_messages();
    log_info("millis_init passed\r\n");
    
    // bring up the GPIO pins and SPI HW interface
    if (init_spi1(NRF8001_SPI, SPI_BaudRatePrescaler_32) != E_SUCCESS) {
        log_err("GPIO and SPI HW bringup failed");
        return -1;
    }

    log_info("init_spi1 passed\r\n");

    nrf8001_setup();
    log_info("nrf8001_setup passed\r\n");

    while(1) {
        //printf("millis_now = %d, millis_old = %d\r\n", millis(), lastUpdate);
        if (Rx_Flag_read) {
            printf("Rx_Flag_read is high\r\n");
            rx_count++;

            stringComplete = true;
            Rx_Flag_read = 0;

            printf("Sending: %s\r\n", (char *)&uart_buffer[0]);

            uart_buffer_len = stringIndex + 1;
            uart_buffer_len = RXBUFFERSIZE;

            if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, uart_buffer, uart_buffer_len))
            {
                printf("Serial input dropped");
            }

            // clear the uart_buffer:
            for (stringIndex = 0; stringIndex < 20; stringIndex++)
            {
                uart_buffer[stringIndex] = ' ';
            }

            // reset the flag and the index in order to receive more data
            stringIndex    = 0;
            stringComplete = false;
        }

        aci_loop();
    }
}