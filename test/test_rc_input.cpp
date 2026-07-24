#include <gtest/gtest.h>

#include <limits>
#include <memory>

#include <lib/CtrlFSM.hpp>

namespace
{

class RCInputTest : public ::testing::Test
{
protected:
    static void SetUpTestCase()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestCase()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override
    {
        node = std::make_shared<rclcpp::Node>("rc_input_test");
    }

    px4_msgs::msg::RcChannels::SharedPtr valid_message() const
    {
        auto message = std::make_shared<px4_msgs::msg::RcChannels>();
        message->channels.fill(0.0F);
        message->channel_count = 10;
        message->signal_lost = false;
        return message;
    }

    rclcpp::Node::SharedPtr node;
    Param_t param;
};

TEST_F(RCInputTest, NoFirstFrameIsInvalid)
{
    CtrlFSM fsm(param, node);
    auto now = node->now();

    EXPECT_FALSE(fsm.rc_data.has_received);
    EXPECT_FALSE(fsm.rc_data.check_validity());
    EXPECT_FALSE(fsm.rc_is_received(now));
}

TEST_F(RCInputTest, SignalLostFrameIsInvalid)
{
    RC_Data_t rc(node);
    auto message = valid_message();
    message->signal_lost = true;

    rc.feed(message, param);

    EXPECT_TRUE(rc.has_received);
    EXPECT_FALSE(rc.check_validity());
    EXPECT_FALSE(rc.is_hover_mode);
    EXPECT_FALSE(rc.is_offboard);
}

TEST_F(RCInputTest, InsufficientChannelCountIsInvalid)
{
    RC_Data_t rc(node);
    auto message = valid_message();
    message->channel_count = 9;

    rc.feed(message, param);

    EXPECT_FALSE(rc.check_validity());
}

TEST_F(RCInputTest, ConfiguredChannelIndexOutOfBoundsIsInvalid)
{
    RC_Data_t rc(node);
    auto message = valid_message();
    message->channel_count = message->channels.size();
    param.rc_debug.ch_gear = static_cast<int>(message->channels.size());

    rc.feed(message, param);

    EXPECT_FALSE(rc.check_validity());
}

TEST_F(RCInputTest, StaleFrameIsInvalid)
{
    CtrlFSM fsm(param, node);
    auto message = valid_message();
    fsm.rc_data.feed(message, param);
    auto fresh_time =
        fsm.rc_data.rcv_stamp + rclcpp::Duration::from_seconds(param.msg_timeout.rc / 2.0);
    auto stale_time =
        fsm.rc_data.rcv_stamp + rclcpp::Duration::from_seconds(param.msg_timeout.rc + 0.01);

    EXPECT_TRUE(fsm.rc_is_received(fresh_time));
    EXPECT_FALSE(fsm.rc_is_received(stale_time));
}

TEST_F(RCInputTest, NonFiniteOrOutOfRangeChannelIsInvalid)
{
    RC_Data_t rc(node);
    auto message = valid_message();
    message->channels[2] = std::numeric_limits<float>::quiet_NaN();

    rc.feed(message, param);
    EXPECT_FALSE(rc.check_validity());

    message = valid_message();
    message->channels[param.rc_debug.ch_mode] = 1.5F;
    rc.feed(message, param);
    EXPECT_FALSE(rc.check_validity());
}

TEST_F(RCInputTest, NormalizedChannelsAreMappedWithoutPwmConversion)
{
    RC_Data_t rc(node);
    auto message = valid_message();
    message->channels[0] = 0.0F;
    message->channels[1] = 0.5F;
    message->channels[2] = -0.5F;
    message->channels[param.rc_debug.ch_p] = -1.0F;
    message->channels[param.rc_debug.ch_i] = 0.0F;
    message->channels[param.rc_debug.ch_d] = 1.0F;

    rc.feed(message, param);

    ASSERT_TRUE(rc.check_validity());
    EXPECT_DOUBLE_EQ(rc.ch[0], 0.0);
    EXPECT_NEAR(rc.ch[1], 1.0 / 3.0, 1e-6);
    EXPECT_NEAR(rc.ch[2], -1.0 / 3.0, 1e-6);
    EXPECT_DOUBLE_EQ(rc.p, 0.0);
    EXPECT_DOUBLE_EQ(rc.i, 0.5);
    EXPECT_DOUBLE_EQ(rc.d, 1.0);
}

}  // namespace
