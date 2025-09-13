#include "quad_movements.h"

#include <algorithm>

#include "oscillator.h"

static const char *TAG = "QuadMovements";

Quad::Quad()
{
    is_quad_resting_ = false;
    // 初始化所有舵机管脚为-1（未连接）
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

Quad::~Quad()
{
    DetachServos();
}

unsigned long IRAM_ATTR millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void Quad::Init(int flu, int fru, int fld, int frd,
              int blu, int bru, int bld, int brd)
{
    servo_pins_[FLU] = flu;
    servo_pins_[FRU] = fru;
    servo_pins_[FLD] = fld;
    servo_pins_[FRD] = frd;
    servo_pins_[BLU] = blu;
    servo_pins_[BRU] = bru;
    servo_pins_[BLD] = bld;
    servo_pins_[BRD] = brd;

    AttachServos();
    is_quad_resting_ = false;
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
void Quad::AttachServos()
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (servo_pins_[i] != -1)
        {
            servo_[i].Attach(servo_pins_[i]);
        }
    }
}

void Quad::DetachServos()
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (servo_pins_[i] != -1)
        {
            servo_[i].Detach();
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
void Quad::SetTrims(int flu, int fru, int fld, int frd,
                  int blu, int bru, int bld, int brd)
{
    
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (servo_pins_[i] != -1)
        {
            servo_[i].SetTrim(servo_trim_[0]);
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
void Quad::MoveServos(int time, int servo_target[])
{
    if (GetRestState() == true)
    {
        SetRestState(false);
    }

    final_time_ = millis() + time;
    if (time > 10)
    {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (servo_pins_[i] != -1)
            {
                increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10.0);
            }
        }

        for (int iteration = 1; millis() < final_time_; iteration++)
        {
            partial_time_ = millis() + 10;
            for (int i = 0; i < SERVO_COUNT; i++)
            {
                if (servo_pins_[i] != -1)
                {
                    servo_[i].SetPosition(servo_[i].GetPosition() + increment_[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    else
    {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (servo_pins_[i] != -1)
            {
                servo_[i].SetPosition(servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
    }

    // final adjustment to the target.
    bool f = true;
    int adjustment_count = 0;
    while (f && adjustment_count < 10)
    {
        f = false;
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (servo_pins_[i] != -1 && servo_target[i] != servo_[i].GetPosition())
            {
                f = true;
                break;
            }
        }
        if (f)
        {
            for (int i = 0; i < SERVO_COUNT; i++)
            {
                if (servo_pins_[i] != -1)
                {
                    servo_[i].SetPosition(servo_target[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            adjustment_count++;
        }
    };
}

void Quad::MoveSingle(int position, int servo_number)
{
    if (position > 180)
        position = 90;
    if (position < 0)
        position = 90;

    if (GetRestState() == true)
    {
        SetRestState(false);
    }

    if (servo_number >= 0 && servo_number < SERVO_COUNT && servo_pins_[servo_number] != -1)
    {
        servo_[servo_number].SetPosition(position);
    }
}

void Quad::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                           double phase_diff[SERVO_COUNT], float cycle = 1)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (servo_pins_[i] != -1)
        {
            servo_[i].SetO(offset[i]);
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetT(period);
            servo_[i].SetPh(phase_diff[i]);
        }
    }

    double ref = millis();
    double end_time = period * cycle + ref;

    while (millis() < end_time)
    {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (servo_pins_[i] != -1)
            {
                servo_[i].Refresh();
            }
        }
        vTaskDelay(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Quad::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                   double phase_diff[SERVO_COUNT], float steps = 1.0)
{
    if (GetRestState() == true)
    {
        SetRestState(false);
    }

    int cycles = (int)steps;

    //-- Execute complete cycles
    if (cycles >= 1)
        for (int i = 0; i < cycles; i++) {
            OscillateServos(amplitude, offset, period, phase_diff);
            // ESP_LOGI("Quad", "Cycle iteration %d", i);
        }
            
    //-- Execute the final not complete cycle
    // OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

///////////////////////////////////////////////////////////////////
//-- HOME = Quad at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Quad::Home()
{
    if (is_quad_resting_ == false)
    { // Go to rest position only if necessary
        // 为所有舵机准备初始位置值
        int homes[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            homes[i] = 90;
        }
        MoveServos(500, homes);
    }

    is_quad_resting_ = true;
}

bool Quad::GetRestState()
{
    return is_quad_resting_;
}

void Quad::SetRestState(bool state)
{
    is_quad_resting_ = state;
}

///////////////////////////////////////////////////////////////////
//-- PREDETERMINED MOTION SEQUENCES -----------------------------//
///////////////////////////////////////////////////////////////////

void Quad::Forward(float steps=3, int period=800)
{
    int x_amp = 15;
    int z_amp = 15;
    int ap = 10;
    int hi = 15;
    int front_x = 6;
    int A[SERVO_COUNT] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
    int O[SERVO_COUNT] = {
             + ap - front_x,
             - ap + front_x,
             - hi,
             + hi,
             - ap - front_x,
             + ap + front_x,
             + hi,
             - hi
        };
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(90), DEG2RAD(90), 
                                      DEG2RAD(180), DEG2RAD(180), DEG2RAD(90), DEG2RAD(90)};

    //-- Let's oscillate the servos!
    Execute(A, O, period, phase_diff, steps);
}

void Quad::Backward(float steps=3, int period=800)
{
    int x_amp = 15;
    int z_amp = 15;
    int ap = 10;
    int hi = 15;
    int front_x = 6;
    int A[SERVO_COUNT] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
    int O[SERVO_COUNT] = {
             + ap - front_x,
             - ap + front_x,
             - hi,
             + hi,
             - ap - front_x,
             + ap + front_x,
             + hi,
             - hi
        };
    double phase_diff[SERVO_COUNT] = {DEG2RAD(180), DEG2RAD(180), DEG2RAD(90), DEG2RAD(90), 
                                      0, 0, DEG2RAD(90), DEG2RAD(90)};

    //-- Let's oscillate the servos!
    Execute(A, O, period, phase_diff, steps);
}

void Quad::EnableServoLimit(int diff_limit)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (servo_pins_[i] != -1)
        {
            servo_[i].SetLimiter(diff_limit);
        }
    }
}

void Quad::DisableServoLimit()
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (servo_pins_[i] != -1)
        {
            servo_[i].DisableLimiter();
        }
    }
}
