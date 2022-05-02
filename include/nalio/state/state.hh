#ifndef NALIO_STATE_STATE_HH__
#define NALIO_STATE_STATE_HH__

#include <Eigen/Core>

namespace nalio {

template <int GLOBAL_SIZE, int LOCAL_SIZE> 
class state {
public:
  using StateT = Eigen::Matrix<double, GLOBAL_SIZE, 1>;
  using InputT = Eigen::Matrix<double, LOCAL_SIZE, 1>;

  virtual void oplus(const InputT& input) = 0;

  StateT get() { return state_; }
  virtual void reset() { state_.setZero(); }
protected:
  StateT state_;
};

}

#endif // NALIO_STATE_STATE_HH__