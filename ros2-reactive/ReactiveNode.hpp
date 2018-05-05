#pragma once

#ifndef REACTIVENODE_HPP
#define REACTIVENODE_HPP

#include <rclcpp/node.hpp>
#include <rxcpp/rx.hpp>

#include <functional>
#include <memory>
#include <string>

namespace ros2_reactive
{

/**
 * Wrapper for ROS2 node, which extends classic ROS2 node with functionality responsible for
 * creating reactive observables.
 * More information about reactive interface can be found in {@link ReactiveX.RxCpp}.
 * @see [ReactiveX.RxCpp](https://github.com/ReactiveX/RxCpp)
 */
class ReactiveNode : public rclcpp::Node
{
 public:
  /**
   * Using-declaration moving rclcpp::Node constructors into this scope.
   * Class can be constructed directly from any rclcpp::Node constructor.
   */
  using rclcpp::Node::Node;

  /**
   * Creates reactive observable from given topic.
   *
   * Function creates standard ROS2 subscription in given topic
   * and wrapps it in RxCpp observable. Created observable can be used with
   * all reactive operators available in rxcpp library to operate on received
   * from ROS2 data. Created observable is inactive, as long as subscribe function
   * will be called on it. It means, that creation is not equivalent with performing
   * subscription.
   *
   * @tparam MessageType Type of ROS2 message in given topic.
   * @tparam TopicType Representation of ROS2 topic
   *     (same as in rclcpp::Node::create_subscription).
   * @tparam Args All additional parameters which will be forwarded to
   *     rclcpp::Node::create_subscription function.
   *
   * @param topic ROS2 topic (same as in rclcpp::Node::create_subscription).
   * @param args All additional parameters of rclcpp::Node::create_subscription
   *     function, which will be forwarded (apart from topic and callback) to
   *     create_subscription function.
   *
   * @return Reactive observable of given MessageType and topic.
   */
  template <typename MessageType, typename TopicType, typename... Args>
  rxcpp::observable<MessageType> create_observable(TopicType&& topic,
                                                   Args&&... args)
  {
    return rxcpp::observable<>::create<MessageType>(
        [&,
         subscription = typename rclcpp::Subscription<MessageType>::SharedPtr(nullptr),
         subject = rxcpp::subjects::subject<MessageType>{}]
        (rxcpp::subscriber<MessageType> subscriber)
        {
          const_cast<typename rclcpp::Subscription<MessageType>::SharedPtr&>(
              subscription) =
              rclcpp::Node::create_subscription<MessageType>(
                  std::forward<TopicType>(topic),
                  [subject](typename MessageType::UniquePtr msg) {
                    subject.get_subscriber().on_next(*msg);
                  },
                  std::forward<Args>(args)...);
          return subject.get_observable().subscribe(subscriber);
        });
  }
};
}  // namespace ros2_reactive

#endif  // REACTIVENODE_HPP
