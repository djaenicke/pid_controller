#ifndef PID_H_
#define PID_H_

namespace pid {

class PID {
 private:
  float kp_;
  float ki_;
  float kd_;
  float dt_;
  float tol_; /* Tolerance */
  float last_e_;
  float integral_;
 public:
  PID(float kp, float ki, float kd, float dt, float tol = 0.0f);
  float Step(float sp, float fb, float max, float min);
  void  Reset(void);
};

}  // namespace pid

#endif  // PID_H_
