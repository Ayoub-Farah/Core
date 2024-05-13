/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author ClÃ©ment Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"

//-------------- ZEPHYR INCLUDE ----------------------------------
#include "zephyr/console/console.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); //code to be executed in the slow communication task
void loop_application_task();   //code to be executed in the fast application task
void loop_control_task();       //code to be executed in real-time at 20kHz

//--------------USER VARIABLES DECLARATIONS-------------------

enum serial_interface_menu_mode //LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE =0,
    POWERMODE,
};

uint8_t received_serial_char;
uint8_t mode = IDLEMODE;

static float32_t duty_cycle = 0.1; //[-] duty cycle (comm task)
static bool pwm_enable_prim = false; //[bool] state of the PWM (ctrl task)
static bool pwm_enable_sec = false; //[bool] state of the PWM (ctrl task)
static uint32_t control_task_period = 100; //[us] period of the control task
static float32_t phase_shift = 0;

static float32_t V1_low_value; //store value of V1_low (app task)
static float32_t V2_low_value; //store value of V2_low (app task)
static float32_t VIIhigh_value; //store value of Vhigh (app task)

static float32_t IIIhigh_value; //store value of i1_low (app task)
static float32_t II_high_value; //store value of i2_low (app task)
static float32_t VIhigh_value; //store value of ihigh (app task)

static float32_t meas_data; //temp storage meas value (ctrl task)

static bool soft_start = false;

//--------------SETUP FUNCTIONS-------------------------------

/**
 * @brief Initialiaze the bridge primary side (low voltage)
*/
void init_BridgePrim()
{
    spin.gpio.configurePin(PC12, OUTPUT);

    /* LEG 1 TIMER A initialization */
    spin.pwm.setModulation(PWMA, UpDwn); // Set modulation
    spin.pwm.setAdcEdgeTrigger(PWMA, EdgeTrigger_up); // Configure ADC rollover in center aligned mode
    spin.pwm.setSwitchConvention(PWMA, PWMx1); // choose which output of the timer unit to control whith duty cycle
    spin.pwm.setMode(PWMA, VOLTAGE_MODE);
    spin.pwm.initUnit(PWMA); // Initialize leg unit
    spin.pwm.setDeadTime(PWMA, 200, 200); // Configure PWM dead time
    spin.pwm.setAdcDecimation(PWMA, 1);
    spin.pwm.setAdcTrigger(PWMA, ADCTRIG_1);
    spin.pwm.enableAdcTrigger(PWMA);

    /* LEG2 TIMER C initialization */
    spin.pwm.setModulation(PWMC, UpDwn); // Set modulation
    spin.pwm.setAdcEdgeTrigger(PWMC, EdgeTrigger_up); // Configure ADC rollover in center aligned mode
    spin.pwm.setSwitchConvention(PWMC, PWMx1); // choose which output of the timer unit to control whith duty cycle
    spin.pwm.setMode(PWMC, VOLTAGE_MODE);
    spin.pwm.initUnit(PWMC); // Initialize leg unit
    spin.pwm.setDeadTime(PWMC, 200, 200); // Configure PWM dead time
    spin.pwm.setPhaseShift(PWMC, 180);
    spin.pwm.setAdcDecimation(PWMC, 1);
    spin.pwm.setAdcTrigger(PWMC, ADCTRIG_3);
    spin.pwm.enableAdcTrigger(PWMC);
}

/**
 * @brief Initialiaze the bridge secondary side (low voltage)
*/
void init_BridgeSec()
{
    spin.gpio.configurePin(PC13, OUTPUT);

    /* LEG 1 TIMER E initialization */
    spin.pwm.setModulation(PWME, UpDwn); // Set modulation
    spin.pwm.setSwitchConvention(PWME, PWMx1); // choose which output of the timer unit to control whith duty cycle
    spin.pwm.setMode(PWME, VOLTAGE_MODE);
    spin.pwm.initUnit(PWME); // Initialize leg unit
    spin.pwm.setDeadTime(PWME, 100, 100); // Configure PWM dead time
    spin.pwm.setPhaseShift(PWME, 0);

    /* LEG2 TIMER F initialization */
    spin.pwm.setModulation(PWMF, UpDwn); // Set modulation
    spin.pwm.setSwitchConvention(PWMF, PWMx2); // choose which output of the timer unit to control whith duty cycle
    spin.pwm.setMode(PWMF, VOLTAGE_MODE);
    spin.pwm.initUnit(PWMF); // Initialize leg unit
    spin.pwm.setDeadTime(PWMF, 100, 100); // Configure PWM dead time
    spin.pwm.setPhaseShift(PWMF, 0);
}

/**
 * @brief start bridge primary side
*/
void start_BridgePrim()
{
    spin.gpio.setPin(PC12);
    spin.pwm.startDualOutput(PWMA);
    spin.pwm.startDualOutput(PWMC);
}

/**
 * @brief start bridge secondary side
*/
void start_BridgeSec()
{
    spin.gpio.setPin(PC13);
    spin.pwm.startDualOutput(PWME);
    spin.pwm.startDualOutput(PWMF);
}

/**
 * @brief stop bridge primary side
*/
void stop_BridgePrim()
{
    spin.pwm.stopDualOutput(PWMA);
    spin.pwm.stopDualOutput(PWMC);
    spin.gpio.resetPin(PC12);
}

/**
 * @brief stop bridge secondary side
*/
void stop_BridgeSec()
{
    spin.pwm.stopDualOutput(PWME);
    spin.pwm.stopDualOutput(PWMF);
    spin.gpio.resetPin(PC13);
}

/**
 * @brief set duty cycle bridge primary side
*/
void setDuty_BridgePrim(float32_t duty_cycle)
{
    spin.pwm.setDutyCycle(PWMA, duty_cycle);
    spin.pwm.setDutyCycle(PWMC, duty_cycle);
}

/**
 * @brief set duty cycle bridge secondary side
*/
void setDuty_BridgeSec(float32_t duty_cycle)
{
    spin.pwm.setDutyCycle(PWME, duty_cycle);
    spin.pwm.setDutyCycle(PWMF, duty_cycle);
}

/**
 * @brief set phase shift bridge secondary side
*/
void setPhaseShift_BridgeSec(float32_t phaseShift_deg)
{
    spin.pwm.setPhaseShift(PWME, phaseShift_deg);
    spin.pwm.setPhaseShift(PWMF, phaseShift_deg);
}

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    console_init();

    // Setup the hardware first
    spin.version.setBoardVersion(TWIST_v_1_1_2);

    data.enableTwistDefaultChannels();
    data.setParameters(I_HIGH, 0.030, 0); // Calibrate VIhigh value
    data.setParameters(V_HIGH, 0.13, 0); // Calibrate VIIhigh value

    spin.pwm.setFrequency(200000); // Configure PWM frequency to 200kHz

    init_BridgePrim();
    init_BridgeSec();

    // Then declare tasks
    uint32_t communication_task_number = task.createBackground(loop_communication_task);
    uint32_t application_task_number = task.createBackground(loop_application_task);
    task.createCritical(loop_control_task, 100); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(communication_task_number);
    task.startBackground(application_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------

void loop_communication_task()
{
    while(1) {
        received_serial_char = console_getchar();
        switch (received_serial_char) {
            case 'h':
                //----------SERIAL INTERFACE MENU-----------------------
                printk(" ________________________________________\n");
                printk("|     ------- MENU ---------             |\n");
                printk("|     press i : idle mode                |\n");
                printk("|     press s : Start secondary side     |\n");
                printk("|     press p : Start primary side       |\n");
                printk("|     press u : phase shift UP           |\n");
                printk("|     press d : phase shift DOWN         |\n");
                printk("|________________________________________|\n\n");
                //------------------------------------------------------
                break;
            case 'i':
                printk("idle mode\n");
                mode = IDLEMODE;
                break;
            case 'p':
                printk("power mode\n");
                mode = POWERMODE;
                break;
            case 's':
                printk("second bridge start \n");
                pwm_enable_prim = true;
            case 'u':
                phase_shift += 0.5;
                break;
            case 'd':
                phase_shift -= 0.5;
                break;
            default:
                break;
        }
    }
}

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
    printk("%f:", phase_shift);
    printk("%f:", VIhigh_value);
    printk("%f:", VIIhigh_value);
    printk("%f:", IIIhigh_value);
    printk("%f\n", II_high_value);
    // Pause between two runs of the task
    task.suspendBackgroundMs(1000);
}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_control_task()
{

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != -10000)
        VIIhigh_value = meas_data;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != -10000)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != -10000)
        V2_low_value = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != -10000)
        VIhigh_value = meas_data;

    meas_data = data.getLatest(I1_LOW);
    if (meas_data != -10000)
        IIIhigh_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != -10000)
        II_high_value = meas_data;

    if(mode == IDLEMODE)
    {
        if(pwm_enable_sec)
        {
            stop_BridgePrim();
            stop_BridgeSec();
            pwm_enable_sec = false;
            pwm_enable_prim = false;
            duty_cycle = 0.1;
        }
    }
    else if(mode == POWERMODE)
    {
        // Soft start to 0.5 duty_cycle
        if(duty_cycle < 0.5)
        {
            duty_cycle += 0.05;
        }
        else 
        {
            duty_cycle = 0.5;

            if(pwm_enable_prim)
            {   
                setDuty_BridgePrim(duty_cycle);
                start_BridgePrim();
            }

            setPhaseShift_BridgeSec(phase_shift);
        }
        setDuty_BridgeSec(duty_cycle);
        if(!pwm_enable_sec)
        {
            pwm_enable_sec = true;
            start_BridgeSec();
        }
    }

}

/**
 * This is the main function of thPWM1is example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
