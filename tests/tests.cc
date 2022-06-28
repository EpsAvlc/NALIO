#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#include "datahub/datahub.hh"
#include "nalio/ceres/loam_factors.hh"
#include "nalio/factory/factory.hh"
#include "nalio/propagator/linear_propagator.hh"

TEST(NALIO, FACTORY) {
  class Base {
   public:
    virtual int calc() { return 5; }
  };

  class Derived : public Base {
   public:
    virtual int calc() { return 10; }
  };

  using namespace nalio;
  REGISTER_NALIO(Base, Derived, "TEST_Derived")
  auto derived_ptr = nalio::Factory<Base>::produce_unique("TEST_Derived");
  EXPECT_NE(derived_ptr, nullptr);
  EXPECT_EQ(derived_ptr->calc(), 10);
  auto derived_ptr2 = nalio::Factory<Base>::produce_unique("TEST_Derived2");
  EXPECT_EQ(derived_ptr2, nullptr);
}

struct LidarEdgeFactor {
  LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                  Eigen::Vector3d last_point_b_, double s_)
      : curr_point(curr_point_),
        last_point_a(last_point_a_),
        last_point_b(last_point_b_),
        s(s_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()),
                               T(last_point_a.z())};
    Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()),
                               T(last_point_b.z())};

    // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) *
    // q[2]};
    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
    q_last_curr = q_identity.slerp(T(s), q_last_curr);
    Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = q_last_curr * cp + t_last_curr;
    std::cout << "lp: " << lp.transpose() << std::endl;

    Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
    std::cout << "nu: " << nu.transpose() << std::endl;
    Eigen::Matrix<T, 3, 1> de = lpa - lpb;
    std::cout << "de: " << de.norm() << std::endl;

    residual[0] = nu.x() / de.norm();
    residual[1] = nu.y() / de.norm();
    residual[2] = nu.z() / de.norm();

    // residual[0] = T(0);
    // residual[1] = T(0);
    // residual[2] = T(0);
    return true;
  }

 private:
  Eigen::Vector3d curr_point, last_point_a, last_point_b;
  double s;
};

TEST(NALIO, Factor) {
  NalioPoint ori{0, 1, 1};
  NalioPoint neigh_a{0, 1, 2};
  NalioPoint neigh_b{0, 2, 1};

  nalio::LOAMEdgePair edge_pair;
  edge_pair.ori_pt = ori;
  edge_pair.neigh_pt[0] = neigh_a;
  edge_pair.neigh_pt[1] = neigh_b;

  nalio::LOAMEdgeFactor factor(edge_pair);
  double q[4]{0, 0, 0, 1};
  double t[3]{0, 0, 0};
  double residual[3] = {0, 0, 0};
  factor(q, t, residual);
  for (int i = 0; i < 3; ++i) {
    std::cout << residual[i] << std::endl;
  }

  LidarEdgeFactor lf(ori.getVector3fMap().cast<double>(),
                     neigh_a.getVector3fMap().cast<double>(),
                     neigh_b.getVector3fMap().cast<double>(), 1);
  double q2[4]{0, 0, 0, 1};
  lf(q2, t, residual);
  for (int i = 0; i < 3; ++i) {
    std::cout << residual[i] << std::endl;
  }
}

// TEST(NALIO, LOAM_STATE) {
//   nalio::LOAMState state;
//   state.reset();
//   nalio::LOAMState::InputT input;
//   input << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
//   state.oplus(input);

//   nalio::LOAMState::StateT expect_state;
//   expect_state << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 1;
//   EXPECT_EQ(state.get(), expect_state);
// }

// TEST(NALIO, LINEAR_PROPAGATOR) {
//   nalio::LinearPropagator propagator;
//   Eigen::Isometry3d p0, p1, p2;
//   p0.setIdentity();
//   EXPECT_TRUE(propagator.propagate().isApprox(p0));
//   propagator.update(p0);
//   propagator.propagate();
//   EXPECT_TRUE(propagator.propagate().isApprox(p0));

//   p1.linear() =
//       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0,
//       1)).toRotationMatrix();
//   p1.translation() = Eigen::Vector3d(10, 10, 10);
//   propagator.update(p1);
//   Eigen::Isometry3d est_prop1;
//   est_prop1.linear() =
//       Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
//   est_prop1.translation() = Eigen::Vector3d(20, 20, 20);
//   EXPECT_TRUE(propagator.propagate().isApprox(est_prop1));
// }

// class DatahubTest : public ::testing::Test {
//  protected:
//   void SetUp() override {
//     running_ = true;
//     buffer_.registerMessage("/sensor/IMU", 400);
//     buffer_.registerMessage("/sensor/Image", 40);
//     buffer_.registerMessage("/sensor/LiDAR", 20);
//     buffer_.registerMessage("/egopose", 400);

//     threads_[0] = std::thread(&DatahubTest::msgLoop, this, "/sensor/IMU",
//     200); threads_[1] = std::thread(&DatahubTest::msgLoop, this,
//     "/sensor/Image", 20); threads_[2] = std::thread(&DatahubTest::msgLoop,
//     this, "/sensor/LiDAR", 10); threads_[3] =
//     std::thread(&DatahubTest::msgLoop, this, "/egopose", 200);
//   }

//   void TearDown() override {
//     running_ = false;
//     for (size_t ti = 0; ti < 4; ++ti) {
//       if (threads_[ti].joinable()) {
//         threads_[ti].join();
//       }
//     }
//   }

//   void msgLoop(const std::string& msg_name, double freq) {
//     uint duration = 1000. / freq;
//     while (running_) {
//       std::chrono::time_point<std::chrono::steady_clock> t_n =
//           std::chrono::steady_clock::now();
//       nalio::Message::Ptr msg(new nalio::Message);
//       msg->name = msg_name;
//       msg->header.timestamp =
//           std::chrono::time_point_cast<std::chrono::microseconds>(t_n)
//               .time_since_epoch()
//               .count();
//       buffer_.receiveMessage(msg);
//       std::chrono::time_point<std::chrono::steady_clock> t_e =
//           t_n + std::chrono::milliseconds(duration);
//       std::this_thread::sleep_until(t_e);
//     }
//   }

//   std::chrono::_V2::steady_clock::time_point awakeTime(uint16_t m) {
//     return std::chrono::steady_clock::now() + std::chrono::milliseconds(m);
//   }

//   nalio::DataBuffer buffer_;
//   bool running_;
//   std::thread threads_[4];
// };

// TEST_F(DatahubTest, Nearest) {
//   std::vector<std::string> msg_names = {"/sensor/LiDAR", "/sensor/Image"};
//   std::vector<nalio::DataSyncer::SyncType> msg_sync_types = {
//       nalio::DataSyncer::SyncType::kNearest,
//       nalio::DataSyncer::SyncType::kNearest};
//   nalio::DataSyncer::Ptr lidar_img_syncer =
//       buffer_.createDataSyncer(msg_names, msg_sync_types);
//   std::vector<std::vector<nalio::Message::Ptr>> synced_msgs;
//   std::vector<int64_t> lidar_times;
//   std::vector<int64_t> image_times;
//   for (uint i = 0; i < 10; ++i) {
//     while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
//       std::this_thread::yield();
//     }
//     lidar_times.push_back(synced_msgs[0][0]->header.timestamp.usec());
//     image_times.push_back(synced_msgs[0][0]->header.timestamp.usec());

//     std::cout << "[----------] lidar time: "
//               << synced_msgs[0][0]->header.timestamp.usec()
//               << ", image time:" <<
//               synced_msgs[1][0]->header.timestamp.usec()
//               << std::endl;
//   }
//   for (int i = 0; i < lidar_times.size() - 1; ++i) {
//     EXPECT_TRUE(lidar_times[i + 1] - lidar_times[i] < 0.15 * 1000000);
//   }

//   for (int i = 0; i < lidar_times.size(); ++i) {
//     EXPECT_TRUE(std::abs(lidar_times[i] - image_times[i]) < 0.1 * 1000000);
//   }
// }

// TEST_F(DatahubTest, Bilateral) {
//   std::vector<std::string> msg_names = {"/sensor/LiDAR", "/egopose"};
//   std::vector<nalio::DataSyncer::SyncType> msg_sync_types = {
//       nalio::DataSyncer::SyncType::kNearest,
//       nalio::DataSyncer::SyncType::kBilateral};
//   nalio::DataSyncer::Ptr lidar_img_syncer =
//       buffer_.createDataSyncer(msg_names, msg_sync_types);
//   std::vector<std::vector<nalio::Message::Ptr>> synced_msgs;
//   std::vector<int64_t> pivot_timestamps;
//   for (uint i = 0; i < 10; ++i) {
//     while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
//       std::this_thread::yield();
//     }
//     pivot_timestamps.push_back(synced_msgs[0][0]->header.timestamp.usec());
//     std::cout << "[----------] lidar time: "
//               << synced_msgs[0][0]->header.timestamp.usec()
//               << ", ego time:" <<
//               synced_msgs[1][0]->header.timestamp.usec()
//               << ", " << synced_msgs[1][1]->header.timestamp.usec()
//               << std::endl;
//     EXPECT_TRUE(synced_msgs[1][0]->header.timestamp.usec() <
//                     synced_msgs[0][0]->header.timestamp.usec() &&
//                 synced_msgs[1][1]->header.timestamp.usec() >
//                     synced_msgs[0][0]->header.timestamp.usec());
//   }
// }

// TEST_F(DatahubTest, Multiple) {
//   std::vector<std::string> msg_names = {"/sensor/LiDAR", "/sensor/IMU"};
//   std::vector<nalio::DataSyncer::SyncType> msg_sync_types = {
//       nalio::DataSyncer::SyncType::kNearest,
//       nalio::DataSyncer::SyncType::kMultiple};
//   nalio::DataSyncer::Ptr lidar_img_syncer =
//       buffer_.createDataSyncer(msg_names, msg_sync_types);
//   std::vector<std::vector<nalio::Message::Ptr>> synced_msgs;
//   std::vector<int64_t> pivot_timestamps;
//   for (uint i = 0; i < 10; ++i) {
//     while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
//       std::this_thread::yield();
//     }
//     pivot_timestamps.push_back(synced_msgs[0][0]->header.timestamp.usec());
//     std::cout << "[----------] lidar time: "
//               << synced_msgs[0][0]->header.timestamp.usec()
//               << ", imu size:" << synced_msgs[1].size() << std::endl;
//   }
// }

// TEST_F(DatahubTest, LVIO) {
//   std::vector<std::string> msg_names = {"/sensor/LiDAR", "/sensor/Image",
//                                         "/sensor/IMU"};
//   std::vector<nalio::DataSyncer::SyncType> msg_sync_types = {
//       nalio::DataSyncer::SyncType::kNearest,
//       nalio::DataSyncer::SyncType::kNearest,
//       nalio::DataSyncer::SyncType::kMultiple};
//   nalio::DataSyncer::Ptr lidar_img_syncer =
//       buffer_.createDataSyncer(msg_names, msg_sync_types);
//   std::vector<std::vector<nalio::Message::Ptr>> synced_msgs;
//   std::vector<int64_t> pivot_timestamps;
//   for (uint i = 0; i < 10; ++i) {
//     while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
//       std::this_thread::yield();
//     }
//     pivot_timestamps.push_back(synced_msgs[0][0]->header.timestamp.usec());
//     std::cout << "[----------] lidar time: "
//               << synced_msgs[0][0]->header.timestamp.usec()
//               << ", Image time: " <<
//               synced_msgs[1][0]->header.timestamp.usec()
//               << ", imu size:" << synced_msgs[2].size() << std::endl;
//   }
// }
