#pragma once

#include <shared_mutex>



enum class State
{
  UNINITIALIZED = 0,
  INITIALIZED = 1,
  ARMED = 2,
  TAKEOFF = 3,
  LAND = 4,
  HOVER_ONBOARD = 5,
  HOVER_OFFBOARD = 6,
  POSITION = 7
};




class QuadState
{
public:

  inline QuadState() {};

  inline void setState(const State &state)
  {
    // exclusive lock for writing 
    mutex_.lock();
    
    state_ = state;
    
    mutex_.unlock();
  }

private:

  State state_{State::UNINITIALIZED};

  std::shared_mutex mutex_;

};