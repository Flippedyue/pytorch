/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "src/CuptiPMSamplingController.h"

using namespace KINETO_NAMESPACE;
using namespace std::chrono_literals;

namespace {

class FakeCuptiPMSamplingApi final : public CuptiPMSamplingApi {
 public:
  void configure(const CuptiPMSamplingConfig& config) override {
    receivedConfig = config;
    if (failConfiguration) {
      throw std::runtime_error("PM sampling configuration failed");
    }
  }

  void start() override {
    ADD_FAILURE() << "Unexpected sampling start";
  }

  bool decode(std::vector<CuptiPMSample>&) override {
    ADD_FAILURE() << "Unexpected sampling decode";
    return true;
  }

  void stop() override {
    ADD_FAILURE() << "Unexpected sampling stop";
  }

  void disable() override {
    ++disableCalls;
  }

  std::optional<CuptiPMSamplingConfig> receivedConfig;
  bool failConfiguration{false};
  int disableCalls{0};
};

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
  const CuptiPMSamplingConfig config{
      0, {"sm__cycles_active.avg"}, 1ms, 1s};
  auto api = std::make_unique<FakeCuptiPMSamplingApi>();
  api->failConfiguration = true;
  const FakeCuptiPMSamplingApi& fakeApi = *api;
  CuptiPMSamplingController controller(config, std::move(api));

  EXPECT_FALSE(controller.prepare());

  ASSERT_TRUE(fakeApi.receivedConfig.has_value());
  EXPECT_EQ(fakeApi.disableCalls, 1);
  EXPECT_FALSE(controller.stop());
  EXPECT_EQ(fakeApi.disableCalls, 1);
}
