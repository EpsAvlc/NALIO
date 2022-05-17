#include "nalio/utils/read_write_mutex.hh"

#include <thread>

namespace nalio {

ReadWriteMutex::ReadWriteMutex() : indicator_(0) {}

void ReadWriteMutex::lockReader() {
  if (indicator_.load() < 0) {
    std::this_thread::yield();
  }
  ++indicator_;
}

void ReadWriteMutex::unlockReader() { --indicator_; }

void ReadWriteMutex::lockWriter() {
  if (indicator_.load() != 0) {
    std::this_thread::yield();
  }
  indicator_.store(-1);
}

void ReadWriteMutex::unlockWriter() { indicator_.store(0); }

ReaderLockGuard::ReaderLockGuard(ReadWriteMutex& mutex) : mutex_(mutex) {
  mutex_.lockReader();
}

ReaderLockGuard::~ReaderLockGuard() { mutex_.unlockReader(); }

WriterLockGuard::WriterLockGuard(ReadWriteMutex& mutex) : mutex_(mutex) {
  mutex_.lockWriter();
}

WriterLockGuard::~WriterLockGuard() { mutex_.unlockWriter(); }

}  // namespace nalio
