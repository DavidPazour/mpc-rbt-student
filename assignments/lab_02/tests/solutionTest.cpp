// clang-format off
#include <thread>

#include <gtest/gtest.h>

#include <mpc-rbt-solution/Receiver.hpp>
#include <mpc-rbt-solution/Sender.hpp>
// clang-format on

TEST(solutionTest, sender)
{
  static constexpr uint8_t testIterations = 50;
  static constexpr uint16_t testPort = 6666;

  std::thread thr([] {
    Sender::Node(Utils::Config::Sender{
                   .localPort = 5555,
                   .remotePort = testPort,
                   .remoteAddress = "127.0.0.1",
                   .sendingPeriodMillis = 10,
                 })
      .run();
  });
  thr.detach();

  Socket::UDP receiver(testPort);
  receiver.create();
  receiver.configure();
  receiver.bind();

  Utils::Message lastMessage{.x = UINT32_MAX, .y = UINT32_MAX, .z = UINT32_MAX};
  for (auto _ = 0; _ < testIterations; ++_) {
    Socket::IPFrame frame{};
    Utils::Message message{};
    ASSERT_TRUE(receiver.receive(frame));
    ASSERT_TRUE(Utils::Message::deserialize(frame, message));
    EXPECT_FALSE(message.frame.empty());
    EXPECT_NE(message.x, lastMessage.x);
    EXPECT_NE(message.y, lastMessage.y);
    EXPECT_NE(message.z, lastMessage.z);
    lastMessage = message;
  }

  if (thr.joinable()) thr.join();
}

TEST(solutionTest, receiver)
{
  static constexpr uint8_t testIterations = 50;
  static constexpr uint16_t testPort = 6667;

  Receiver::TestMock node(Utils::Config::Receiver{.localPort = testPort});
  std::thread thr([&node] { node.run(); });
  thr.detach();

  Socket::UDP sender(5556);
  sender.create();
  sender.configure();
  sender.bind();

  for (auto i = 0; i < testIterations; ++i) {
    Socket::IPFrame frame{.port = testPort, .address = "127.0.0.1"};
    Utils::Message message{
      .frame = "test_frame",
      .x = static_cast<double>(i),
      .y = static_cast<double>(i),
      .z = static_cast<double>(i),
    };
    ASSERT_TRUE(Utils::Message::serialize(frame, message));
    ASSERT_TRUE(sender.send(frame));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(node.getData().frame, message.frame);
    EXPECT_EQ(node.getData().x, message.x);
    EXPECT_EQ(node.getData().y, message.y);
    EXPECT_EQ(node.getData().z, message.z);
  }

  if (thr.joinable()) thr.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
