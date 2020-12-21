#ifndef PID_H_
#define PID_H_

namespace pid
{
typedef struct
{
  float kp;
  float ki;
  float kd;
} Gains;

class PID
{
 private:
  float kp_;
  float ki_;
  float kd_;
  float tol_; /* Tolerance */
  float last_e_;
  float integral_;

 public:
  PID(float tol = 0.0f);
  float step(float sp, float fb, float dt, float max, float min);
  void reset(Gains* gains);
};

}  // namespace pid

#endif  // PID_H_
