#ifndef NALIO_UTILS_SHARED_MUTES__
#define NALIO_UTILS_SHARED_MUTES__

#include <atomic>

namespace nalio {

class ReadWriteMutex {
 public:
  ReadWriteMutex();

  void lockReader();
  void unlockReader();
  void lockWriter();
  void unlockWriter();

 private:
  std::atomic_int16_t indicator_;
};

class ReaderLockGuard {
 public:
  ReaderLockGuard(ReadWriteMutex& mutex);

  ~ReaderLockGuard();

 private:
  ReadWriteMutex& mutex_;
};

class WriterLockGuard {
 public:
  WriterLockGuard(ReadWriteMutex& mutex);

  ~WriterLockGuard();

 private:
  ReadWriteMutex& mutex_;
};

}  // namespace nalio

#endif
