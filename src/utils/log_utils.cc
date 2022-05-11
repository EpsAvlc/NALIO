#include "nalio/utils/log_utils.hh"

namespace nalio {
std::string cutParenthesesNTail(std::string &&pretty_func) {
  size_t pos = pretty_func.find('(');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin() + pos, pretty_func.end());
  }

  pos = pretty_func.find(' ');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin(), pretty_func.begin() + pos + 1);
  }

  pretty_func = "[" + pretty_func + "]";
  return pretty_func;
}
}  // namespace nalio
