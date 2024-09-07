#include <MQUnifiedsensor.h>
/************************Hardware Related Macros************************************/
#define Board ("Arduino UNO")
#define Pin (A1)  // Analog input 1 of your arduino
/***********************Software Related Macros************************************/
#define Type ("MQ-4")  // MQ4
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (10)  // For arduino UNO/MEGA/NANO
#define RatioMQ4CleanAir (4.4)   // RS / R0 = 60 ppm
/*****************************Globals***********************************************/
// Declare Sensor
MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

void setup_mq4() {
    // Init the serial port communication - to debug the library
    // Serial.begin(9600);
    // Set math model to calculate the PPM concentration and the value of constants
    MQ4.setRegressionMethod(1);  //_PPM =  a*ratio^b
    MQ4.setA(1012.7);
    MQ4.setB(-2.786);  // Configure the equation to to calculate CH4 concentration
    /*
      Exponential regression:
    Gas    | a      | b
    LPG    | 3811.9 | -3.113
    CH4    | 1012.7 | -2.786
    CO     | 200000000000000 | -19.05
    Alcohol| 60000000000 | -14.01
    smoke  | 30000000 | -8.308
    */

    /*****************************  MQ Init ********************************************/
    // Remarks: Configure the pin of arduino as input.
    /************************************************************************************/
    MQ4.init();
    /*
      //If the RL value is different from 10K please assign your RL value with the following method:
      MQ4.setRL(20);
    */
    /*****************************  MQ CAlibration ********************************************/
    // Explanation:
    // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
    // and on clean air (Calibration conditions), setting up R0 value.
    // We recomend executing this routine only on setup in laboratory conditions.
    // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
    // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
    // Serial.print("Calibrating please wait.");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++) {
        MQ4.update();  // Update data, the arduino will read the voltage from the analog pin
        calcR0 += MQ4.calibrate(RatioMQ4CleanAir);
        //Serial.print(".");
    }
    MQ4.setR0(calcR0 / 10);
    // Serial.println("  done!.");

    if (isinf(calcR0)) {
        Serial.println(F("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"));
        while (1)
            ;
    }
    if (calcR0 == 0) {
        Serial.println(F("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"));
        while (1)
            ;
    }
    /*****************************  MQ CAlibration ********************************************/
    MQ4.serialDebug(true);
}

float read_mq4() {

    MQ4.update();       // Update data, the arduino will read the voltage from the analog pin
    
    float mq4_val = MQ4.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    Serial.print(F("MQ4. ppm "));
    Serial.print(mq4_val);
    Serial.println(" ");

    return mq4_val;
}
