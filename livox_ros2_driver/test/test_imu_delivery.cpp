#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <future>
#include <thread>

#include "include/livox_ros2_driver.h"

namespace livox_ros {
namespace {

using namespace std::chrono_literals;
using RawImu =
    std::array<uint8_t, KEthPacketHeaderLength + sizeof(LivoxImuPoint)>;

RawImu MakeImu(uint64_t timestamp_ns) {
  RawImu bytes{};
  auto *packet = reinterpret_cast<LivoxEthPacket *>(bytes.data());
  packet->version = 5;
  packet->data_type = kImu;
  packet->timestamp_type = kTimestampTypeNoSync;
  std::memcpy(packet->timestamp, &timestamp_ns, sizeof(timestamp_ns));
  const LivoxImuPoint value{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  std::memcpy(packet->data, &value, sizeof(value));
  return bytes;
}

void EnqueueImu(Lds &source, uint64_t timestamp_ns) {
  auto bytes = MakeImu(timestamp_ns);
  source.StorageRawPacket(0, reinterpret_cast<LivoxEthPacket *>(bytes.data()));
}

TEST(ImuQueue, OwnsBytesAndPreservesTimestampsWithoutClouds) {
  Lds source(100, kSourceRawLidar);
  EnqueueImu(source, 1000000000);
  EnqueueImu(source, 1005000000);
  EXPECT_EQ(source.lidars_[0].data.storage_packet, nullptr);
  ImuPacket first;
  ImuPacket second;
  ASSERT_TRUE(source.PopImuPacket(0, &first));
  ASSERT_TRUE(source.PopImuPacket(0, &second));
  EXPECT_EQ(first.timestamp_ns, 1000000000U);
  EXPECT_EQ(second.timestamp_ns - first.timestamp_ns, 5000000U);
  LivoxImuPoint value;
  const auto *raw =
      reinterpret_cast<const LivoxEthPacket *>(first.storage.raw_data);
  std::memcpy(&value, raw->data, sizeof(value));
  EXPECT_FLOAT_EQ(value.gyro_x, 1.0f);
  EXPECT_FLOAT_EQ(value.acc_z, 6.0f);
  EXPECT_FALSE(source.PopImuPacket(0, &second));
}

TEST(ImuQueue, ReportsOverflowAndResetIsSafe) {
  Lds source(100, kSourceRawLidar);
  for (uint32_t i = 0; i < kImuEthPacketQueueSize + 7; ++i) {
    EnqueueImu(source, 1000000000ULL + i * 5000000ULL);
  }
  EXPECT_EQ(source.GetImuQueueSize(0), kImuEthPacketQueueSize);
  EXPECT_EQ(source.GetImuDroppedPackets(0), 7U);
  source.ResetLidar(&source.lidars_[0], kSourceRawLidar);
  EXPECT_EQ(source.GetImuQueueSize(0), 0U);
  EnqueueImu(source, 3000000000ULL);
  ImuPacket packet;
  ASSERT_TRUE(source.PopImuPacket(0, &packet));
  EXPECT_EQ(packet.timestamp_ns, 3000000000ULL);
}

TEST(ImuQueue, ConcurrentProducerConsumerPreserveOrderAcrossWraparound) {
  Lds source(100, kSourceRawLidar);
  constexpr uint32_t count = 10000;
  std::thread producer([&source]() {
    for (uint32_t i = 0; i < count; ++i) {
      while (source.GetImuQueueSize(0) == kImuEthPacketQueueSize) {
        std::this_thread::yield();
      }
      EnqueueImu(source, 1000000000ULL + i * 5000000ULL);
    }
  });
  for (uint32_t i = 0; i < count; ++i) {
    ImuPacket packet;
    while (!source.PopImuPacket(0, &packet)) {
      std::this_thread::yield();
    }
    EXPECT_EQ(packet.timestamp_ns, 1000000000ULL + i * 5000000ULL);
  }
  producer.join();
  EXPECT_EQ(source.GetImuDroppedPackets(0), 0U);
}

TEST(ImuWakeup, CoalescesSignalsAndWakesBothConsumersOnExit) {
  Lds source(100, kSourceRawLidar);
  for (int i = 0; i < 1000; ++i) {
    source.imu_semaphore_.SignalOnce();
  }
  EXPECT_EQ(source.imu_semaphore_.GetCount(), 1);
  source.imu_semaphore_.Wait();
  auto cloud_wait =
      std::async(std::launch::async, [&source]() { source.semaphore_.Wait(); });
  auto imu_wait = std::async(std::launch::async,
                             [&source]() { source.imu_semaphore_.Wait(); });
  source.RequestExit();
  EXPECT_EQ(cloud_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(imu_wait.wait_for(1s), std::future_status::ready);
  cloud_wait.get();
  imu_wait.get();
}

class ImuDistributor : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ImuDistributor, PublishesWithoutAnyLidarPackets) {
  Lds source(100, kSourceRawLidar);
  auto node = std::make_shared<rclcpp::Node>("test_imu_distributor");
  std::string frame = "livox_frame";
  Lddc distributor(kLivoxCustomMsg, 0, kSourceRawLidar, kOutputToRos, 10,
                   frame);
  distributor.SetRosNode(node.get());
  distributor.RegisterLds(&source);
  auto consumer = std::async(std::launch::async, [&distributor]() {
    distributor.DistributeImuData();
  });
  EnqueueImu(source, 1000000000);
  const auto status = consumer.wait_for(2s);
  EXPECT_EQ(status, std::future_status::ready);
  if (status != std::future_status::ready) {
    source.RequestExit();
  }
  consumer.get();
  EXPECT_EQ(source.GetImuQueueSize(0), 0U);
  EXPECT_EQ(source.lidars_[0].data.storage_packet, nullptr);
}

TEST_F(ImuDistributor, DriverCanUnloadAndReloadWithoutIncomingTraffic) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("user_config_path", TEST_CONFIG_PATH),
       rclcpp::Parameter("cmdline_input_bd_code", "000000000000001")});
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 2; ++i) {
    auto driver = std::make_shared<LivoxDriver>(options);
    std::this_thread::sleep_for(20ms);
    driver.reset();
  }
  EXPECT_LT(std::chrono::steady_clock::now() - start, 3s);
}

}  // namespace
}  // namespace livox_ros
