#include "nalio/utils/read_write_mutex.hh"

#include <thread>

namespace nalio {

ReadWriteMutex::ReadWriteMutex() : reader_num_(0) {}

void ReadWriteMutex::lockReader() {
  read_mutex_.lock();
  ++reader_num_;
  if (reader_num_ == 1) {
    write_mutex_.lock();
  }
  read_mutex_.unlock();
}

void ReadWriteMutex::unlockReader() {
  read_mutex_.lock();
  --reader_num_;
  if (reader_num_ == 0) {
    write_mutex_.unlock();
  }
  read_mutex_.unlock();
}

void ReadWriteMutex::lockWriter() { write_mutex_.lock(); }

void ReadWriteMutex::unlockWriter() { write_mutex_.unlock(); }

ReaderLockGuard::ReaderLockGuard(ReadWriteMutex& mutex) : mutex_(mutex) {
  mutex_.lockReader();
}

ReaderLockGuard::~ReaderLockGuard() { mutex_.unlockReader(); }

WriterLockGuard::WriterLockGuard(ReadWriteMutex& mutex) : mutex_(mutex) {
  mutex_.lockWriter();
}

WriterLockGuard::~WriterLockGuard() { mutex_.unlockWriter(); }

}  // namespace nalio
