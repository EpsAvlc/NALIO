#include "nalio/data/datahub.hh"

#include <iterator>
#include <thread>

#include "nalio/utils/log_utils.hh"

namespace nalio {

DataBuffer::DataBuffer() {
  prune_buffs_thread_ = std::thread(&DataBuffer::pruneBuffs, this);
  running_ = true;
}

DataBuffer::~DataBuffer() {
  running_ = false;
  if (prune_buffs_thread_.joinable()) {
    prune_buffs_thread_.join();
  }
}

DataSyncer::Ptr DataBuffer::createDataSyncer(
    const std::vector<std::string>& message_names,
    const std::vector<DataSyncer::SyncType> sync_types) {
  DataSyncer::Ptr ret(new DataSyncer(message_names, sync_types, this));
  data_syncers_.emplace_back(ret);
  return ret;
}

void DataBuffer::registerMessage(const std::string& message_name,
                                 const size_t buff_size) {
  MessageInfo message_info;
  message_info.id = message_buffs_.size();
  message_info.buff_size = buff_size;
  message_info.name = message_name;

  message_ids_[message_name] = message_info.id;

  message_infos_.emplace_back(message_info);
  message_buffs_.emplace_back(std::list<Message::Ptr>());
  Message::Ptr tmp_msg(new Message);
  tmp_msg->name = message_name;
  tmp_msg->timestamp = -1;
  // To make message_buffs[x].begin() != message_buffs[x].end()
  message_buffs_.back().emplace_back(tmp_msg);
}

void DataBuffer::receiveMessage(const Message::Ptr& message) {
  ReaderLockGuard lock(mutex_);
  uint16_t message_id;
  if (!getMessageId(message->name, &message_id)) {
    ROS_ERROR_STREAM_FUNC("Message " << message->name
                                     << " has not been registered.");
    throw(std::invalid_argument("Message has not been registered."));
  }

  if (!message_buffs_[message_id].empty() &&
      message->timestamp < message_buffs_[message_id].back()->timestamp) {
    ROS_ERROR_STREAM_FUNC("Disorder messages: "
                          << message->name << ", current mesaage timestamp: "
                          << message->timestamp << ", last message timestamp: "
                          << message_buffs_[message_id].back()->timestamp);
    throw(std::logic_error("Disorder messages."));
  }
  message_buffs_[message_id].push_back(message);
}

bool DataBuffer::getMessageId(const std::string& message_name,
                              uint16_t* message_id) {
  if (message_ids_.find(message_name) == message_ids_.end()) {
    return false;
  } else {
    *message_id = message_ids_.at(message_name);
    return true;
  }
}

void DataBuffer::pruneBuffs() {
  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    WriterLockGuard lock(mutex_);
    for (int mi = 0; mi < message_buffs_.size(); ++mi) {
      if (message_infos_[mi].buff_size < message_buffs_.size()) {
        std::string message_name = message_infos_[mi].name;
        auto new_begin_iter = message_buffs_[mi].begin();
        std::advance(new_begin_iter,
                     message_buffs_.size() - message_infos_[mi].buff_size);
        for (int si = 0; si < data_syncers_.size(); ++si) {
          uint16_t syncer_message_id;
          if (!data_syncers_[si]->getMessageId(message_name,
                                               &syncer_message_id)) {
            continue;
          }
          if ((*data_syncers_[si]->message_iters_[syncer_message_id])
                  ->timestamp > (*new_begin_iter)->timestamp) {
            continue;
          }
          data_syncers_[si]->message_iters_[syncer_message_id] = new_begin_iter;
        }

        message_buffs_[mi].erase(message_buffs_[mi].begin(), new_begin_iter);
      }
    }
  }
}

DataBuffer::IteratorType DataBuffer::messageBuffEnd(const uint16_t id) {
  return message_buffs_[id].end();
}

DataSyncer::DataSyncer(const std::vector<std::string>& message_names,
                       const std::vector<SyncType> sync_types,
                       DataBuffer* buffer)
    : buffer_(buffer) {
  ReaderLockGuard lock(buffer->mutex_);
  for (size_t mi = 0; mi < message_names.size(); ++mi) {
    if (message_names.size() != sync_types.size()) {
      ROS_ERROR_STREAM_FUNC("Message names' size must equal sync types size.");
      throw(std::invalid_argument(
          "Message names' size must equal sync types size."));
    }
    uint16_t buffer_message_id;
    if (!buffer_->getMessageId(message_names[mi], &buffer_message_id)) {
      ROS_ERROR_STREAM_FUNC("Message has not been registrated "
                            << message_names[mi]);
      throw(std::invalid_argument("Message has not been registrated."));
    }

    MessageInfo info;
    info.id = message_infos_.size();
    info.sync_type = sync_types[mi];
    info.name = message_names[mi];

    buffer_message_ids_.emplace_back(buffer_message_id);
    message_infos_.emplace_back(info);
    message_iters_.emplace_back(
        buffer_->message_buffs_[buffer_message_id].begin());
  }
}

bool DataSyncer::getSyncMessages(
    std::vector<std::vector<Message::Ptr>>* synced_msgs, const double t_thres) {
  if (nullptr == synced_msgs) {
    ROS_ERROR_STREAM_FUNC("synced_msgs is nullptr!");
    throw(std::invalid_argument("synced_msgs is nulptr!"));
  }

  synced_msgs->clear();
  synced_msgs->resize(message_infos_.size());

  ReaderLockGuard lock(buffer_->mutex_);
  uint16_t buffer_pivot_msg_id;
  if (!buffer_->getMessageId(message_infos_[0].name, &buffer_pivot_msg_id)) {
    throw(std::invalid_argument("message " + message_infos_[0].name +
                                " has not been registered."));
  }
  std::vector<DataBuffer::IteratorType> new_iters(message_infos_.size());
  new_iters[0] = message_iters_[0];
  // skip head node.
  if (std::distance(new_iters[0],
                    buffer_->messageBuffEnd(buffer_pivot_msg_id)) == 1) {
    return false;
  }
  ++new_iters[0];

  synced_msgs->at(0).emplace_back(*new_iters[0]);
  int64_t pivot_timestamp = (*new_iters[0])->timestamp;
  int64_t tl = pivot_timestamp - static_cast<int64_t>(t_thres * 1000000);
  int64_t tr = pivot_timestamp + static_cast<int64_t>(t_thres * 1000000);

  for (int mi = 1; mi < message_infos_.size(); ++mi) {
    if (SyncType::kNearest == message_infos_[mi].sync_type.val ||
        SyncType::kBilateral == message_infos_[mi].sync_type.val) {
      uint16_t msg_id = message_infos_[mi].id;
      DataBuffer::IteratorType search_begin_it = message_iters_[msg_id];
      uint16_t buffer_msg_id = buffer_message_ids_.at(msg_id);
      DataBuffer::IteratorType search_end_it =
          buffer_->messageBuffEnd(buffer_msg_id);
      std::vector<Message::Ptr> between_msgs;
      DataBuffer::IteratorType ret_iter = buffer_->getMessagesBetween(
          search_begin_it, search_end_it, tl, tr, &between_msgs);
      if (ret_iter == search_end_it) {
        return false;
      }
      new_iters[mi] = ret_iter;
      if (SyncType::kNearest == message_infos_[mi].sync_type.val) {
        auto nearest_iter = std::min_element(
            between_msgs.begin(), between_msgs.end(),
            [&pivot_timestamp](Message::Ptr& lhs, Message::Ptr& rhs) {
              return std::abs(lhs->timestamp - pivot_timestamp) <
                     std::abs(rhs->timestamp - pivot_timestamp);
            });
        synced_msgs->at(mi).push_back(*nearest_iter);
      } else {
        // skip head node.
        if (buffer_->message_buffs_[buffer_message_ids_[mi]].size() > 1 &&
            (*message_iters_[mi])->timestamp < 0) {
          ++message_iters_[mi];
        }
        // increate pivot iterator to make it between two other msgs.
        if ((*message_iters_[mi])->timestamp > pivot_timestamp) {
          message_iters_[0] = new_iters[0];
          return false;
        }
        size_t lhs_ind = 0;
        for (; lhs_ind < between_msgs.size() - 1; ++lhs_ind) {
          if (between_msgs[lhs_ind]->timestamp < pivot_timestamp &&
              between_msgs[lhs_ind + 1]->timestamp > pivot_timestamp) {
            break;
          }
        }
        if (between_msgs.size() - 1 == lhs_ind) {
          return false;
        }
        synced_msgs->at(mi).push_back(between_msgs[lhs_ind]);
        synced_msgs->at(mi).push_back(between_msgs[lhs_ind + 1]);
      }
    } else if (message_infos_[mi].sync_type.val == SyncType::kMultiple) {
      uint16_t msg_id = message_infos_[mi].id;
      DataBuffer::IteratorType search_begin_it = message_iters_[msg_id];
      uint16_t buffer_msg_id = buffer_message_ids_.at(msg_id);
      DataBuffer::IteratorType search_end_it =
          buffer_->messageBuffEnd(buffer_msg_id);
      std::vector<Message::Ptr> before_msgs;
      DataBuffer::IteratorType ret_iter = buffer_->getMessagesBefore(
          search_begin_it, search_end_it, pivot_timestamp, &before_msgs);
      if (ret_iter == search_end_it) {
        return false;
      }
      new_iters[mi] = ret_iter;
      for (size_t bi = 0; bi < before_msgs.size(); ++bi) {
        synced_msgs->at(mi).push_back(before_msgs.at(bi));
      }
    } else {
      throw std::invalid_argument("unknown sync type");
    }
  }
  // update iterators
  for (int ni = 0; ni < new_iters.size(); ++ni) {
    message_iters_[ni] = new_iters[ni];
  }
  // release atomic variable
  return true;
}

bool DataSyncer::getMessageId(const std::string& message_name,
                              uint16_t* message_id) {
  if (message_ids_.find(message_name) == message_ids_.end()) {
    return false;
  } else {
    *message_id = message_ids_.at(message_name);
    return true;
  }
}

DataBuffer::IteratorType DataBuffer::getMessagesBetween(
    const IteratorType& iter_begin, const IteratorType& iter_end,
    const int64_t tl, const int64_t tr,
    std::vector<Message::Ptr>* synced_msgs) {
  IteratorType ret = iter_end;
  if (nullptr == synced_msgs) {
    ROS_ERROR_STREAM_FUNC("synced_msg is nullptr!");
    throw(std::invalid_argument("synced_msg is nullptr."));
  }
  synced_msgs->clear();
  for (auto mit = iter_begin; mit != iter_end; ++mit) {
    if ((*mit)->timestamp > tl && (*mit)->timestamp < tr) {
      if (ret == iter_end) {
        ret = mit;
      }
      synced_msgs->push_back(*mit);
    }
    if ((*mit)->timestamp > tr) {
      break;
    }
  }

  return ret;
}

DataBuffer::IteratorType DataBuffer::getMessagesBefore(
    const IteratorType& iter_begin, const IteratorType& iter_end,
    const int64_t tr, std::vector<Message::Ptr>* before_msgs) {
  if (nullptr == before_msgs) {
    throw(std::invalid_argument("before_msgs is nullptr."));
  }
  before_msgs->clear();
  IteratorType ret = iter_end;
  for (auto iter = iter_begin; iter != iter_end; ++iter) {
    if ((*iter)->timestamp < tr) {
      before_msgs->push_back(*iter);
      ret = iter;
    } else {
      break;
    }
  }
  return ret;
}

};  // namespace nalio
