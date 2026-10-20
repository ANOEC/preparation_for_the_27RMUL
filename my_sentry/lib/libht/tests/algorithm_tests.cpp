#include <gtest/gtest.h>
#include <limits>
#include "pid/PID.hpp"
#include "leso/leso.hpp"
#include "ladrc/ladrc.hpp"
#include "rls/RLS.hpp"
#include "wave/sine.hpp"

TEST(StandaloneAlgorithms, PidComputesOutputAndRejectsInvalidStep)
{
  PID::Param param{};
  param.kp = 2.0f;
  PID::Controller controller(param);
  ASSERT_EQ(controller.update(3.0f, 1.0f, 0.01f), Algorithm::ErrorCode::OK);
  EXPECT_FLOAT_EQ(controller.getOutput(), 4.0f);
  EXPECT_EQ(controller.update(9.0f, 1.0f, 0.0f), Algorithm::ErrorCode::ARG_ERR);
  EXPECT_FLOAT_EQ(controller.getOutput(), 4.0f);
}

TEST(StandaloneAlgorithms, ObserverInitializesFromFeedback)
{
  LESO::Observer<2> observer(1.0f, 10.0f);
  ASSERT_EQ(observer.update(0.0f, 3.0f, 0.01f), Algorithm::ErrorCode::OK);
  EXPECT_FLOAT_EQ(observer.getStatus()[0], 3.0f);
  EXPECT_EQ(observer.update(0.0f, 4.0f, -1.0f), Algorithm::ErrorCode::ARG_ERR);
  EXPECT_FLOAT_EQ(observer.getStatus()[0], 3.0f);
}

TEST(StandaloneAlgorithms, LadrcUsesObserverAndOwnErrorType)
{
  LADRC::Controller<1> controller(1.0f, 2.0f, 10.0f);
  ASSERT_EQ(controller.update(3.0f, 1.0f, 0.01f), Algorithm::ErrorCode::OK);
  EXPECT_FLOAT_EQ(controller.getOutput(), 4.0f);
}

TEST(StandaloneAlgorithms, RlsFitsLineAndPreservesStateOnInvalidSample)
{
  RLS::RLS<2> estimator(1000.0f, 1.0f);
  for (unsigned i = 0; i < 200; ++i)
  {
    Eigen::Vector2f sample(1.0f, static_cast<float>(i % 7));
    ASSERT_EQ(estimator.update(sample, 2.0f - 0.5f * sample[1]), Algorithm::ErrorCode::OK);
  }
  EXPECT_NEAR(estimator.getParamsVector()[0], 2.0f, 0.001f);
  EXPECT_NEAR(estimator.getParamsVector()[1], -0.5f, 0.001f);
  const auto before = estimator.getParamsVector();
  const auto covariance = estimator.getCovarianceMatrix();
  const Eigen::Vector2f invalid(1.0f, std::numeric_limits<float>::quiet_NaN());
  EXPECT_EQ(estimator.update(invalid, 1.0f), Algorithm::ErrorCode::ARG_ERR);
  EXPECT_TRUE(estimator.getParamsVector() == before);
  EXPECT_TRUE(estimator.getCovarianceMatrix() == covariance);
}

TEST(StandaloneAlgorithms, SineUsesStandaloneConfiguration)
{
  Wave::SineWaveGenerator generator(1.0f, 2.0f, 1.0f);
  ASSERT_EQ(generator.update(0.25f), Algorithm::ErrorCode::OK);
  EXPECT_NEAR(generator.getOutput(), 3.0f, 0.00001f);
}
