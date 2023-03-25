#ifndef NALIO_FACTORY_FACTORY_HH__
#define NALIO_FACTORY_FACTORY_HH__

#include <functional>
#include <unordered_map>
#include <memory>
#include <string>
#include "nalio/system/system.hh"

namespace nalio {
template <typename Base>
class Factory {
 public:
  static std::unordered_map<std::string, std::function<Base*()>> &map() {
    static std::unordered_map<std::string, std::function<Base*()>> ret;
    return ret;
  }

  template <typename Derived>
  struct register_t {
    register_t(const std::string& key) {
      Factory::get().map().emplace(key, [] { return new Derived(); });
    }

    template <typename... Args>
    register_t(const std::string& key, Args... args) {
      Factory::get().map().emplace(key, [&] { return new Derived(args...); });
    }
  };

  static Base* produce(const std::string& key) {
    if (map().find(key) == map().end()) {
      // throw std::invalid_argument("the key is not exist!")
      return nullptr;
    }
    return map()[key]();
  }

  static std::unique_ptr<Base> produce_unique(const std::string& key) {
      return std::unique_ptr<Base>(produce(key));
  }

  static std::shared_ptr<Base> produce_shared(const std::string& key) {
      return std::shared_ptr<Base>(produce(key));
  }

 private:
  Factory(){};
  Factory(const Factory&) = delete;
  Factory(Factory&&) = delete;

  static Factory& get() {
    static Factory instance;
    return instance;
  }
  // static std::unordered_map<std::string, std::function<Base*()>> map_;
};

// template <typename Base>
// std::unordered_map<std::string, std::function<Base*()>> Factory<Base>::map_;

// template <>
// std::map<std::string, std::function<System*()>> Factory<System>::map_ = {};

#define REGISTER_NALIO_VAR(Base, Derived) reg_nalio_##Base##_##Derived##_
#define REGISTER_NALIO(Base, Derived, key, ...)                                \
  static Factory<Base>::register_t<Derived> REGISTER_NALIO_VAR(Base, Derived)( \
      key, ##__VA_ARGS__)

}  // namespace nalio

#endif  // NALIO_FACTORY_FACTORY_HH__
