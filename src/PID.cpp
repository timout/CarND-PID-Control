#include "PID.h"

using namespace std;

Filter * create_pid(PID_TYPE p_type, double Kp, double Ki, double Kd)
{
    switch(p_type) {
        case PID_TYPE::GDBP:
            return new GDBPid(Kp, Ki, Kd);
        case PID_TYPE::TWIDDLE:
            return new TwiddlePid(Kp, Ki, Kd);
        default:
            return new PlainPID(Kp, Ki, Kd);
    }
}

double PlainPID::value(double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    return -Kp*p_error - Kd*d_error - Ki*i_error;
}

/* ----------- GDBPid -------------- */

GDBPid::GDBPid(double kp, double ki, double kd)
{
    this->pid = new PlainPID(kp, ki, kd);
}

GDBPid::~GDBPid()
{
    delete(this->pid);
}

bool GDBPid::is_need_training() 
{
    // stop training if criteria has been met.
    if ( is_trained ) return false;
    current_cumulative_error = sqrt(cumulative_error / epoch_size);
    is_trained = current_cumulative_error < error_threshold;
    if ( is_trained ) {
        cout << "Training is completed. Cumulative error=" << current_cumulative_error << endl;
    }
    return ! is_trained;
}

void GDBPid::reset() 
{
    cumulative_error = 0;
    i_error_abs = 0;
    last_value = 0;
}

double GDBPid::tune(double k, double dx, double delta)
{
    return k - (k * dx * delta * learn_rate);
}

void GDBPid::train() 
{
    double delta = previous_cumulative_error - current_cumulative_error;
    previous_cumulative_error = current_cumulative_error;
    
    double kp = tune(pid->P(), -pid->P_error(), delta);
    double ki = tune(pid->I(), -i_error_abs, delta);
    double kd = tune(pid->D(), -pid->D_error(), delta);
    pid->update(kp, ki, kd);
}

double GDBPid::value(double cte)
{
    last_value = pid->value(cte);
    cumulative_error += cte*cte;
    i_error_abs += fabs(cte);
    step += 1;

    // End of epoch (epoch_size iterations) - time to adjust parameters
    if ( this->step % this->epoch_size == 0 ) {
        if ( is_need_training() ) {
            train();
        } 
        cout << "CTE: " << cte << " Value: " << last_value << " " << *this << endl;
        reset();
    }
    return last_value;
}

void GDBPid::post_process(Sim & sim)
{
    if ( fabs(last_value) > 5 ) {
        reset();
        step = 0;
        current_cumulative_error = 0;
        previous_cumulative_error = 0;
        pid->reset(pid->P(), pid->I(), pid->D());
        sim.reset();
    }
}


/* --------------- Twiddle ----------------- */

TwiddlePid::TwiddlePid(double kp, double ki, double kd)
{
    this->pid = new PlainPID(kp, ki, kd);
}

TwiddlePid::~TwiddlePid()
{
    delete(this->pid);
}

void TwiddlePid::run(double error)
{
    double p[3]{ pid->P(), pid->I(), pid->D() };
	double dp_sum = dp[0] + dp[1] + dp[2];
	if (dp_sum < tolerance) return; // Done with tuning
    switch(state) {
        case INIT:
            p[ci] += dp[ci];
            state = INCREASE;
            if ( best_error < 0 ) best_error = error;
            break;
        case INCREASE:
            if (error < best_error) {
                best_error = error;
                dp[ci] *= 1.1;
                state = INIT;
                ci += 1;
            } else {
                p[ci] -= 2 * dp[ci];
                state = DECREASE;
            }
            break;
        case DECREASE:
            if (error < best_error) {
                best_error = error;
                dp[ci] *= 1.1;
            } else {
                p[ci] += dp[ci];
                dp[ci] *= 0.9;
            }
            state = INIT;
            ci += 1;
            break;
    }
    if (ci == p_size) {
        ci = 0;
        ++iter;
    }
    pid->reset(p[0], p[1], p[2]);
}

double TwiddlePid::value(double cte)
{
    double v = pid->value(cte);
    step += 1;
    total_error += cte*cte;
    // End of epoch (epoch_size iterations) - time to adjust parameters
    if ( this->step == this->epoch_size || ( best_error > 0 && total_error > best_error )) {   
        cout << "CTE: " << cte << " Value: " << v << " " << *this << endl;
        run(total_error);
    }
    return v;
}

void TwiddlePid::post_process(Sim & sim)
{
    if ( this->step == this->epoch_size || ( best_error > 0 && total_error > best_error ) ) {
        sim.reset();
        step = 0;
        total_error = 0;
    }
}