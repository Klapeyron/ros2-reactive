#pragma once

#ifndef REACTIVENODE_HPP
#define REACTIVENODE_HPP

#include <rclcpp/node.hpp>
#include <rxcpp/rx.hpp>

#include <functional>
#include <memory>
#include <string>

class ReactiveNode : public rclcpp::Node
{
 public:
  using rclcpp::Node::Node;

  template <typename MessageType, typename... Args>
  rxcpp::observable<MessageType> create_observable(std::string const& topic,
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
                  topic,
                  [subject](typename MessageType::UniquePtr msg) {
                    subject.get_subscriber().on_next(*msg);
                  },
                  std::forward<Args>(args)...);
          return subject.get_observable().subscribe(subscriber);
        });
  }
};

#endif  // REACTIVENODE_HPP
