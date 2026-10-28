// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <QLabel>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace autoware::intersection_priority
{
class QueuePanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit QueuePanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void subscribe();
  void refresh();
  void show_queue(const diagnostic_msgs::msg::DiagnosticStatus * status);
  QString topic_{"/intersection_priority/queues"};
  QLabel * health_;
  QLabel * title_;
  QLabel * position_;
  std::array<QLabel *, 4> slots_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  struct Inbox
  {
    std::mutex mutex;
    diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message;
    std::chrono::steady_clock::time_point received;
  };
  // Callback owns the inbox, never the QWidget (safe during panel removal).
  std::shared_ptr<Inbox> inbox_;
  QString signature_;
  std::string active_intersection_;
};
}  // namespace autoware::intersection_priority
