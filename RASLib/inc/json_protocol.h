//
// This code is intended to faciliate communication between the microcontroller and a connected computer by 
// providing a subscriber/publisher framework to send and respond to data. Data is sent over UART using JSON 
// encoding.
//
// To start publishing sensor data, first initialize the sensor, then create a handler to stringify the sensor's 
// data, and then call the AddPublisher function with the handler and an optional data pointer. Finally, call the 
// BeginPublishing function to start a periodic timer event to call the handler and publish the data. 
//
// For example, the following code will publish data from two encoders:
//
//     char msgBuffer[20];
//         
//     char* encoderPubHandler(void* data) {
//         SPrintf(msgBuffer, "%d", GetEncoder((tEncoder*)data));
//         
//         return msgBuffer;
//     }
//
//     tPub rightEncPub, leftEncPub;
//
//     tEncoder *rightEnc = InitializeEncoder(...),
//              *leftEnc = InitializeEncoder(...);
//
//     InitializePublisher(&rightEncPub, "right_encoder", rightEnc, encoderPubHandler);
//     InitializePublisher(&leftEncPub, "left_encoder", leftEnc, encoderPubHandler);
//
//     BeginPublishing(.1); 
//
// BeginPublishing will schedule a timer event to publish the following JSON-encoded data over UART every .1 
// seconds:
//
//      {"right_encoder":"<right encoder ticks>","left_encoder":"<left encoder ticks>"}\n
//
// Similarly, the following code will parse messages sent to the microcontroller in order to set motor 
// powers:
//
//      void motorSubHandler(void* data, char* jsonvalue) {
//          int power = atoi(jsonvalue);
//
//          // expecting a power level in the range [-100,100]
//          if (power < -100 || power > 100) {
//              Printf("received unexpected motor power level: %d\n", power);
//              return; 
//          }
//
//          SetMotor((tMotor*)data, power/100.0);
//      }
//
//      tSub rightMotorSub, leftMotorSub;
//
//      tMotor *rightMotor = InitializeMotor(...),
//             *leftMotor = InitializeMotor(...);
//
//      InitializeSubscriber(&rightMotorSub, "right_motor", rightMotor, motorSubHandler);
//      InitializeSubscriber(&leftMotorSub, "left_motor", leftMotor, motorSubHandler);
//
//      BeginSubscribing(.1);
//
// BeginSubscribing will read lines from UART in a loop, trying to parse JSON-encoded objects. If it 
// parses a JSON-encoded message that has either a "motor_right" or "motor_left" field, like this:
//
//      {"right_motor":"<right motor power>","left_motor":"<left motor power>"}\n
//  
// It will then call the motorSubscriber handler for "right_motor" and "left_motor", which will set the 
// left and right motors to the power provided.
//

#define MAX_PUBLISHERS 10
#define MAX_SUBSCRIBERS 10
#define MAX_KEY_LEN 15
#define MAX_VAL_LEN 20
#define MAX_IN_MSG_SIZE 1000
#define MAX_OUT_MSG_SIZE 1000
#define MAX_IN_TOKENS 20

struct {
    char key[MAX_KEY_LEN];
    void *data;
    char* (*handler)(void*);
    unsigned int keylen;
} typedef tPub;

struct {
    char key[MAX_KEY_LEN];
    void *data;
    void (*handler)(void*,char*);
    unsigned int keylen;
} typedef tSub;

// Initializes a handler that will be called to create a string value for the given key, 
//  which will be sent in a json message.
//  (this will deepcopy jsonkey to the key buffer in the pub struct)
int InitializePublisher(tPub *pub, char *jsonkey, void *data, char* (*handler)(void*)); 

// Initializes a handler to be called whenever the given key is found in received messages
//  (this will deepcopy jsonkey to the key buffer in the sub struct)
int InitializeSubscriber(tSub *sub, char *jsonkey, void *data, void (*handler)(void*,char*)); 

// Begins calling all publisher handlers and encoding data every 'period' seconds
void BeginPublishing(float period);

// Begins parsing messages and calling subscriber handlers in a blocking loop, pausing for
//  'secsBetweenReads' seconds between each read
void BeginSubscribing(float secsBetweenReads);

