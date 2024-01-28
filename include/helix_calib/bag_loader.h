#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rosbag/bag.h"
#include "rosbag/message_instance.h"
#include "rosbag/query.h"
#include "rosbag/view.h"

namespace helix
{
class ROSBagLoader
{
public:
  using CallbackFunc = std::function<void(const rosbag::MessageInstance&)>;

private:
  rosbag::Bag bag_;
  std::vector<std::string> topics_;
  std::vector<CallbackFunc> callbacks_;

public:
  ROSBagLoader(const std::string& filepath) : bag_(filepath, rosbag::bagmode::Read)
  {
  }

  void addCallback(const std::string& topic, const CallbackFunc& callback)
  {
    topics_.push_back(topic);
    callbacks_.push_back(callback);
  }

  void load()
  {
    rosbag::View view(bag_, rosbag::TopicQuery(topics_));
    std::size_t size = view.size();
    std::size_t cnt = 0;
    std::size_t permillage = 0;
    std::size_t permillage_curr = 0;
    for (auto& m : view)
    {
      for (std::size_t i = 0; i < topics_.size(); i++)
      {
        if (m.getTopic() == topics_[i])
        {
          callbacks_[i](m);
          break;
        }
      }
      cnt++;
      permillage_curr = cnt * 1000 / size;
      if (permillage_curr > permillage)
      {
        permillage = permillage_curr;
        auto percentage = std::div(permillage, 10);
        std::cout << "Loading bag: " << percentage.quot << '.' << percentage.rem << "%...\r" << std::flush;
      }
    }
  }
};
}  // namespace helix
