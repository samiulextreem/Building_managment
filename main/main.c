// this is a lose implementation of building's main door lock system and communication system with resident system
// the door lock can be opened by using multiple switch  which is located at both inside the home of resident and in front of the gate inside the building 
// the lock should be closed automatically after a certain delat when it is opened
// There should be a calling board outside the building lock which can be used to communicate with the resident.
// The communication process should be independent without any interference from the door locking mechanism


#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/pwm.h"



//referencing MCU pin

// we consider 3 switch to open the main gate lock.
#define lockswitch_1 15
#define lockswitch_2 16
#define lockswitch_3 4

// pin that control the relay that control that lock mechanism
#define lock_motor 5

// as for example, there is two switch which can be used to establish communication with the resident
#define callboard_button_1 12
#define callboard_button_2 13



#define PWM_PERIOD 1000  //pwm period in us




#define BUF_SIZE 1024


#define GPIO_OUTPUT_SEL (1ULL << lock_motor)
#define GPIO_INPUT_SEL ((1ULL << lockswitch_1) | (1ULL << lockswitch_2) | (1ULL << lockswitch_3) | (1ULL << callboard_button_1) |  (1ULL << callboard_button_2))


bool switch_pressed = false; //boolen which is used to get the state of switch
bool lock_open = false;   //boolen which is used to get the state of lock
bool comm_open = false;   //boolen which is used to get the state of communication is established or not



//function decleration
void task_1_lockswitch(void *args); // function to work with switches state
void task_2_locksmotor(void *args); // function to control the relay of lock motor
void task_3_callboard(void *args); // function to deal with calling the resident of the building
void task_4_telecom(void *args); // function to deal with communication pipeline
void task_5_uart_listener(void *args); // since we can not have a actual lock and switch to work with, we are simulating these condition
                                        // using keyboard input with boolen logic



//these are the handles which can be used to deal and indicate any task
static xTaskHandle Handle_task_1;
static xTaskHandle Handle_task_2;
static xTaskHandle Handle_task_3;
static xTaskHandle Handle_task_4;

// initiated three messaging queue to make sure that message can be passd from uart listerner function to their respetive destination function
static xQueueHandle switchstate_que = NULL;
static xQueueHandle communication_que_1 = NULL;
static xQueueHandle communication_que_2 = NULL;



void app_main() {
    vTaskDelay(10);

    //declearing the struct to confugure the io
    gpio_config_t io_conf;
  

    // setting up outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;  //disable interrupt
    io_conf.pin_bit_mask = GPIO_OUTPUT_SEL; // bit mask of the output pins
    io_conf.mode = GPIO_MODE_OUTPUT;        //setting the pins as output
    io_conf.pull_down_en = 0;               //no need to pull or down mode for output pin
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);                  // finally configured the pins with given settings for output
    
    // now we will set up input pins 
    io_conf.intr_type = GPIO_INTR_POSEDGE;      // will register input on positive edge
    io_conf.pin_bit_mask = GPIO_INPUT_SEL;      // bit mask of the input pins
    io_conf.mode = GPIO_MODE_INPUT;             // select the pins as input
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;// enable pulldown mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;   // enable pull up mode
    gpio_config(&io_conf);                       // finally configured the pins with given settings for input


    // setting up the uart communication protocol
    //defining coonfiguration struct
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // initiating the queue system
    switchstate_que = xQueueCreate(1,sizeof(uint8_t)); // queue to pass around data which will indicate the state of the switch
    communication_que_1 = xQueueCreate(1,sizeof(uint8_t)); // communication queue to pass message from calling board to connect to the resident's end
    communication_que_2 = xQueueCreate(1,sizeof(uint8_t));  // queue to let the system know that the resident has terminated connection
    uart_param_config(UART_NUM_0, &uart_config); //configuring the parameter
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0); // finially finishing up the uart initiation


    vTaskDelay(10);

    printf("\n");
    printf("\n");
    printf("\n");
    printf("\n");
    printf("\n");
 

    //finally initializing all the task with xTaskCreate function

    xTaskCreate(task_1_lockswitch,"function for handling switches",2024,NULL,3,&Handle_task_1);
    xTaskCreate(task_3_callboard, "function for all stop command", 2024, NULL, 3, &Handle_task_3);
    xTaskCreate(task_5_uart_listener, "function for getting command from PC", 2024, NULL, 3, NULL);

}


// the aim of this task is to open the lock of the door when one of the multiple switch is pressed
void task_1_lockswitch(void *args){
    uint8_t data;
    for (;;)
    {
        //check if any new message has arrived from the queue
        if(xQueueReceive(switchstate_que, &data , 10) == pdTRUE){
            if (switch_pressed == true && lock_open == false){ // to check if the switch is pressed and door is already closed
                switch_pressed = false;                        //  setting the value of boolen to false cause we dont need this to be true anymore
                printf("Task #1 reporting  -- should open the door lock\n");
                xTaskCreate(task_2_locksmotor, "function for getting the lock open", 2024, NULL, 3, Handle_task_2); // initiated the task which will operate the lock
                
            }
            vTaskDelay(5);
            if (switch_pressed == true && lock_open == true){      // if switch is pressed and door is already open, no need to do anything
                printf("Task #1 reporting -- ##lock is already open\n");
            }
            vTaskDelay(5);
        }
    }
}

void task_2_locksmotor(void *args){
    while (lock_open == false){
        gpio_set_level(lock_motor,1);    // set the pin high which control the relay for lock
        lock_open = true;                // set the boolen to its appropiate state
        printf("Task #2 reporting -- ##  lock state is %d\n", gpio_get_level(lock_motor));
        vTaskDelay(500);
        gpio_set_level(lock_motor,0);    // after a certain delay set the lock control pin low
        printf("Task #2 reporting -- ##  lock state is %d\n", gpio_get_level(lock_motor));
        lock_open = false;               // set the boolen to its appropiate state
        break;
    }
    vTaskDelete(Handle_task_2);          // delete the task after the operation
}

void task_3_callboard(void *args){
    uint8_t data;
    while (1){
        if(xQueueReceive(communication_que_1,&data , 10) == pdTRUE){ //receive message from uart listener
            vTaskDelay(5);
            if (comm_open == false){   // if communication is not opened, then we will proceed to open communication
                comm_open = true;        // set the boolen to its appropiate state
                printf("Task #3 reporting ## establishing communication\n");
                task_4_telecom(NULL);    // call the fucntion which will call the resident end of the communication line
            }
            vTaskDelay(5);
        }
    }
}

void task_4_telecom(void *args){
    uint8_t data;
    printf("task #4 reporting ## communication established \n"); // this function running means that the communication is already established
    while (1) {
        if(xQueueReceive(communication_que_2, &data , 10) == pdTRUE){    //check if there is any message coming from the queue
            comm_open = false;       // set the boolen to its appropiate state
            printf("Task #4 reporting ##  severe communication\n");
            vTaskDelay(5);
            break;                           // break the loop
        }
    }
}




void task_5_uart_listener(void *args){
    uint8_t data;
    while (1)
    {
        uart_read_bytes(UART_NUM_0, &data, BUF_SIZE, 20 / portTICK_RATE_MS); // read the data from the keyboard
        if (data == 's')               // if the received data is 's' , then the lock should open
        {
            if(lock_open == false){                                          // is the lock  already closed
                printf("task #5 : recieved command to open the door  \n");
                switch_pressed = true;
                if(xQueueSend(switchstate_que,&data,10) == pdTRUE){          // send the data to the task_1
                    data = 0;
                }
            }else{                                                            // if the condition does not met, then zero the input data
                data = 0;
            }

            vTaskDelay(5);                                                     
        }
        if (data == 't')                                                      // if the received data is 't' , then the communication should open
        {
            if (comm_open == false){
                printf("task #5 : recieved command to create communication channal \n");
                if(xQueueSend(communication_que_1,&data,10) == pdTRUE){
                    data = 0;
                }
            }else{
                data = 0;
            }
            vTaskDelay(5);

        }

        if (data == 'd')         // if the received data is 't' , then the communication should terminate
        {
            if (comm_open == true){
                printf("task #5 : recieved command to terminate communication channal \n");
                if(xQueueSend(communication_que_2,&data,10) == pdTRUE){  // send the data to the task_1
                    data = 0;
                }
            }else{                        // if the condition does not met, then zero the input data
                data = 0;
            }
            vTaskDelay(5);

        }
        uart_flush(UART_NUM_0);       // flash the uart port
    }
}
