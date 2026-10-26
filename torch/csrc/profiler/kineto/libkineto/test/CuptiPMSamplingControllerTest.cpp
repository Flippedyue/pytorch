/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "src/CuptiPMSamplingController.h"

using namespace KINETO_NAMESPACE;
using namespace std::chrono_literals;

namespace {

struct DecodeBatch {
  std::vector<CuptiPMSample> samples;
  bool drained{true};
};

class FakeCuptiPMSamplingApi final : public ICuptiPMSamplingApi {
 public:
  void configure(const CuptiPMSamplingConfig& config) override {
    ++configureCalls;
    receivedConfig = config;
    if (failConfiguration) {
      throw std::runtime_error("PM sampling configuration failed");
    }
  }

  void start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++startCalls;
    stopped_ = false;
    if (failStart) {
      throw std::runtime_error("PM sampling start failed");
    }
  }

  bool decode(std::vector<CuptiPMSample>& samples) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++decodeCalls_;
    if (stopped_) {
      ++drainCalls;
    }
    decoded_.notify_all();
    if (failDecode) {
      throw std::runtime_error("PM sampling decode failed");
    }
    std::deque<DecodeBatch>& batches = stopped_ ? drainBatches : runningBatches;
    if (batches.empty()) {
      return true;
    }
    const DecodeBatch& batch = batches.front();
    samples.insert(samples.end(), batch.samples.begin(), batch.samples.end());
    const bool drained = batch.drained;
    batches.pop_front();
    return drained;
  }

  void stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stopCalls;
    stopped_ = true;
  }

  void disable() override {
    ++disableCalls;
  }

  bool waitForDecodeCalls(int count) {
    std::unique_lock<std::mutex> lock(mutex_);
    return decoded_.wait_for(lock, 5s, [&] { return decodeCalls_ >= count; });
  }

  int decodeCalls() {
    std::lock_guard<std::mutex> lock(mutex_);
    return decodeCalls_;
  }

  // Set scripts before start(); inspect results after stop() joins the worker.
  std::deque<DecodeBatch> runningBatches;
  std::deque<DecodeBatch> drainBatches;
  std::optional<CuptiPMSamplingConfig> receivedConfig;
  bool failConfiguration{false};
  bool failStart{false};
  bool failDecode{false};
  int configureCalls{0};
  int startCalls{0};
  int stopCalls{0};
  int drainCalls{0};
  int disableCalls{0};

 private:
  std::mutex mutex_;
  std::condition_variable decoded_;
  int decodeCalls_{0};
  bool stopped_{false};
};

const CuptiPMSamplingConfig kConfig{0, {"sm__cycles_active.avg"}, 1ms, 1s};

} // namespace

TEST(CuptiPMSamplingControllerTest, ForwardsConfigurationToApi) {
  const std::vector<CuptiPMSamplingConfig> configs{
      {2, {"sm__cycles_active.avg", "dram__bytes_read.sum"}, std::nullopt, 2s},
      {0, {"sm__cycles_active.avg"}, 1ms, 1s},
  };
  for (const CuptiPMSamplingConfig& config : configs) {
    SCOPED_TRACE(config.deviceId);
    auto api = std::make_unique<FakeCuptiPMSamplingApi>();
    const FakeCuptiPMSamplingApi& fakeApi = *api;
    CuptiPMSamplingController controller(config, std::move(api));

    ASSERT_TRUE(controller.prepare());

    ASSERT_TRUE(fakeApi.receivedConfig.has_value());
    EXPECT_EQ(fakeApi.receivedConfig->deviceId, config.deviceId);
    EXPECT_EQ(fakeApi.receivedConfig->metricNames, config.metricNames);
    EXPECT_EQ(fakeApi.receivedConfig->samplingInterval, config.samplingInterval);
    EXPECT_EQ(fakeApi.receivedConfig->lookbackWindow, config.lookbackWindow);
    EXPECT_FALSE(controller.stop());
    EXPECT_EQ(fakeApi.disableCalls, 1);
  }
}

TEST(CuptiPMSamplingControllerTest, CleansUpWhenConfigurationFails) {
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  api->failConfiguration = true;
  const FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(kConfig, std::move(api));

  EXPECT_FALSE(controller.prepare());

  ASSERT_TRUE(fakeApi.receivedConfig.has_value());
  EXPECT_EQ(fakeApi.disableCalls, 1);
  EXPECT_FALSE(controller.stop());
  EXPECT_EQ(fakeApi.disableCalls, 1);
}

TEST(CuptiPMSamplingControllerTest, RejectsInvalidConfigBeforeCallingApi) {
  const std::vector<CuptiPMSamplingConfig> configs{
      {-1, {"sm__cycles_active.avg"}, 1ms, 1s},
      {0, {}, 1ms, 1s},
      {0, {""}, 1ms, 1s},
      {0, {"sm__cycles_active.avg", ""}, 1ms, 1s},
  };
  for (const CuptiPMSamplingConfig& config : configs) {
    SCOPED_TRACE(testing::PrintToString(config.metricNames));
    auto api = std::make_unique<FakeCuptiPMSamplingApi>();
    const FakeCuptiPMSamplingApi& fakeApi = *api;
    CuptiPMSamplingController controller(config, std::move(api));

    EXPECT_FALSE(controller.prepare());
    controller.start();

    EXPECT_EQ(fakeApi.configureCalls, 0);
    EXPECT_FALSE(fakeApi.receivedConfig.has_value());
    EXPECT_EQ(fakeApi.startCalls, 0);
    EXPECT_FALSE(controller.stop());
    EXPECT_EQ(fakeApi.disableCalls, 0);
  }
}

TEST(CuptiPMSamplingControllerTest, RejectsPreparationUntilStopped) {
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  const FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(kConfig, std::move(api));

  ASSERT_TRUE(controller.prepare());
  EXPECT_FALSE(controller.prepare());
  EXPECT_EQ(fakeApi.configureCalls, 1);
  EXPECT_EQ(fakeApi.disableCalls, 0);

  EXPECT_FALSE(controller.stop());
  EXPECT_EQ(fakeApi.disableCalls, 1);
  EXPECT_TRUE(controller.prepare());
  EXPECT_EQ(fakeApi.configureCalls, 2);
  EXPECT_FALSE(controller.stop());
  EXPECT_EQ(fakeApi.disableCalls, 2);
}

TEST(CuptiPMSamplingControllerTest, FiltersWorkerSamplesAndTransfersOwnership) {
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  api->runningBatches = {
      {{{100, 110, {1.0}}}, false},
      {{{120, 130, {2.0}},
        {140, 150, {}},
        {160, 170, {3.0, 4.0}},
        {190, 180, {5.0}},
        {200, 210, {std::numeric_limits<double>::quiet_NaN()}},
        {220, 230, {std::numeric_limits<double>::infinity()}},
        {240, 250, {-std::numeric_limits<double>::infinity()}},
        {260, 260, {0.0}}},
       true},
  };
  FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(kConfig, std::move(api));

  ASSERT_TRUE(controller.prepare());
  controller.start();
  EXPECT_TRUE(fakeApi.waitForDecodeCalls(2));
  EXPECT_TRUE(controller.stop());

  const std::vector<CuptiPMSample> samples = controller.takeSamples();
  ASSERT_EQ(samples.size(), 2);
  EXPECT_EQ(samples[0].rawStartTimestamp, 120);
  EXPECT_EQ(samples[0].rawEndTimestamp, 130);
  EXPECT_EQ(samples[0].values, (std::vector<double>{2.0}));
  EXPECT_EQ(samples[1].rawStartTimestamp, 260);
  EXPECT_EQ(samples[1].rawEndTimestamp, 260);
  EXPECT_EQ(samples[1].values, (std::vector<double>{0.0}));
  EXPECT_TRUE(controller.takeSamples().empty());
}

TEST(CuptiPMSamplingControllerTest, DrainsUntilAllBatchesAreDecoded) {
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  api->drainBatches = {
      {{{100, 110, {1.0}}}, false},
      {{{120, 130, {2.0}}}, false},
      {{{140, 150, {3.0}}}, true},
  };
  FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(kConfig, std::move(api));

  ASSERT_TRUE(controller.prepare());
  controller.start();
  EXPECT_TRUE(fakeApi.waitForDecodeCalls(1));
  EXPECT_TRUE(controller.stop());

  EXPECT_EQ(fakeApi.startCalls, 1);
  EXPECT_EQ(fakeApi.stopCalls, 1);
  EXPECT_EQ(fakeApi.drainCalls, 3);
  EXPECT_EQ(fakeApi.disableCalls, 1);
  const std::vector<CuptiPMSample> samples = controller.takeSamples();
  ASSERT_EQ(samples.size(), 2);
  EXPECT_EQ(samples[0].rawStartTimestamp, 120);
  EXPECT_EQ(samples[0].rawEndTimestamp, 130);
  EXPECT_EQ(samples[0].values, (std::vector<double>{2.0}));
  EXPECT_EQ(samples[1].rawStartTimestamp, 140);
  EXPECT_EQ(samples[1].rawEndTimestamp, 150);
  EXPECT_EQ(samples[1].values, (std::vector<double>{3.0}));

  EXPECT_FALSE(controller.stop());
  EXPECT_EQ(fakeApi.stopCalls, 1);
  EXPECT_EQ(fakeApi.drainCalls, 3);
  EXPECT_EQ(fakeApi.disableCalls, 1);
}

TEST(CuptiPMSamplingControllerTest, SkipsFinalDrainAfterWorkerFailure) {
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  api->failDecode = true;
  FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(kConfig, std::move(api));

  ASSERT_TRUE(controller.prepare());
  controller.start();
  EXPECT_TRUE(fakeApi.waitForDecodeCalls(1));
  EXPECT_TRUE(controller.stop());

  EXPECT_EQ(fakeApi.decodeCalls(), 1);
  EXPECT_EQ(fakeApi.stopCalls, 1);
  EXPECT_EQ(fakeApi.drainCalls, 0);
  EXPECT_EQ(fakeApi.disableCalls, 1);
  EXPECT_TRUE(controller.takeSamples().empty());
}

TEST(CuptiPMSamplingControllerTest, DisablesWithoutStartingWorkerOnStartFailure) {
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  api->failStart = true;
  FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(kConfig, std::move(api));

  ASSERT_TRUE(controller.prepare());
  EXPECT_NO_THROW(controller.start());
  EXPECT_EQ(fakeApi.startCalls, 1);
  EXPECT_EQ(fakeApi.disableCalls, 1);
  EXPECT_FALSE(controller.stop());
  EXPECT_EQ(fakeApi.stopCalls, 0);
  EXPECT_EQ(fakeApi.decodeCalls(), 0);
  EXPECT_EQ(fakeApi.disableCalls, 1);
}
