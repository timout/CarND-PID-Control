#ifndef PID_H
#define PID_H

#include <iostream>
#include <math.h>

enum PID_TYPE {PLAIN, GDBP, TWIDDLE};

/** In case if we need to talk back to simulation **/
class Sim
{
public:
  virtual ~Sim() = default;

  virtual void reset() = 0; //Reset simulation
};

/** Interface class */
class Filter 
{
public:
  virtual ~Filter() = default;
  virtual double value(double cte) = 0;
  virtual void post_process(Sim & sim) = 0;
  virtual std::ostream&  format(std::ostream & out) const = 0;
  friend std::ostream& operator << (std::ostream & out, const Filter & f);
};

inline std::ostream& operator << (std::ostream& out, const Filter & f) {
    return f.format(out);
}

/** Plain PID implementation */
class PlainPID : public Filter 
{
private:
  double p_error {0};
  double i_error {0};
  double d_error {0};

  /*  Coefficients */ 
  double Kp;
  double Ki;
  double Kd;

public:

  PlainPID(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {};
  virtual ~PlainPID() = default;

  virtual double value(double cte);

  virtual void post_process(Sim & sim) {}

  inline void update(double kp, double ki, double kd) 
  {
    Kp = kp;
    Ki = ki;
    Kd = kd;
  }

  inline void reset(double kp, double ki, double kd) 
  {
    update(kp, ki, kd);
    p_error = 0;
    i_error = 0;
    d_error = 0;
  }

  inline double P() const { return Kp; };
  inline double I() const { return Ki; };
  inline double D() const { return Kd; };

  inline double P_error() const { return p_error; };
  inline double I_error() const { return i_error; };
  inline double D_error() const { return d_error; };

  inline virtual std::ostream&  format(std::ostream & out) const {
    out << " Kp=" << P() << " Ki=" << I() << " Kd=" << D();
    return out;
  };
};

/** Gradient Descent with Backpropagation implementation */
class GDBPid : public Filter
{
private:

  const double error_threshold {0.1}; //when to stop learning
  const double learn_rate {1e-2}; // how fast to learn
  const int epoch_size {220}; // number of iterations per epoch
  
  bool is_trained = false;
  
  PlainPID * pid;

  /* training parameters */
  int step {0}; // iterations counter
  double cumulative_error {0}; // RMSE acumulator (per epoch)
  double i_error_abs {0}; //absolute cte value accumulator (since cte can be positive and negative)
  double current_cumulative_error {0};
  double previous_cumulative_error {0};

  double last_value {0};

public:
  GDBPid(double kp, double ki, double kd);
  virtual ~GDBPid();

  virtual double value(double cte);
  virtual void post_process(Sim & sim);
private:

  /** Determine if training is needed */
  bool is_need_training();
  /** Computes adjustments according to total epoch delta error and their partial derivatives. */
  void train(); 
  /** Tune a coificient 
   * dx: partial derivative for a coificient 
   * de: total delta error over epoch
   */
  double tune(double k, double dx, double ddelta);

  /** Reset epoch accumulators */
  void reset();

public:
  inline virtual std::ostream & format(std::ostream & out) const {
    std::string status = ( is_trained ) ? "Final" : "Train";
    out << status << " rmse=" << current_cumulative_error << " ";
    return pid->format(out);
  };
};

/** Twiddle implementation */
class TwiddlePid : public Filter
{
private:

  enum State
  {
      INIT,
      INCREASE,
      DECREASE,
  };

  const int p_size{3}; //number of coef
  const double tolerance{0.2}; //when to stop learning
  double best_error{-1};
  double total_error{0};
  int iter{0};
  double dp[3]{1.0, 1.0, 1.0};
  int ci{0};
  enum State state;

  const int epoch_size {2000}; // number of iterations per epoch
  
  PlainPID * pid;

  int step {0}; // iterations counter

public:
  TwiddlePid(double kp, double ki, double kd);
  virtual ~TwiddlePid();

  virtual double value(double cte);
  virtual void post_process(Sim & sim);

  inline virtual std::ostream & format(std::ostream & out) const {
    out << "Step: " << iter << " dp: " << dp[0] << ", " << dp[1] << ", " << dp[2];
    out << " best error: " << best_error << " ";
    return pid->format(out);
  };

private:

  void run(double error);
};

Filter * create_pid(PID_TYPE p_type, double Kp, double Ki, double Kd);


#endif /* PID_H */
