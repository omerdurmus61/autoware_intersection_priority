// Copyright 2026 Autoware Contributors
// SPDX-License-Identifier: Apache-2.0
#include "queue_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <algorithm>

namespace autoware::intersection_priority
{
QueuePanel::QueuePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setSpacing(10);
  title_ = new QLabel(this);
  title_->setWordWrap(true);
  title_->setStyleSheet("font-size: 16px; font-weight: bold;");
  layout->addWidget(title_);
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    auto * row = new QHBoxLayout;
    auto * number = new QLabel(QString::number(i + 1), this);
    number->setFixedWidth(24);
    number->setAlignment(Qt::AlignCenter);
    number->setStyleSheet("font-size: 16px; font-weight: bold;");
    row->addWidget(number);
    slots_[i] = new QLabel(this);
    slots_[i]->setObjectName(QString("queue_slot_%1").arg(i + 1));
    slots_[i]->setAlignment(Qt::AlignCenter);
    slots_[i]->setMinimumSize(150, 44);
    slots_[i]->setWordWrap(true);
    row->addWidget(slots_[i], 1);
    layout->addLayout(row);
  }
  position_ = new QLabel(this);
  layout->addWidget(position_);
  health_ = new QLabel("Waiting for queue data", this);
  health_->setWordWrap(true);
  layout->addWidget(health_);
  layout->addStretch();
  show_queue(nullptr);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { refresh(); });
}
void QueuePanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    health_->setText("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  subscribe();
  timer_->start(200);
}
void QueuePanel::subscribe()
{
  if (!node_) {
    return;
  }
  subscription_.reset();
  inbox_ = std::make_shared<Inbox>();
  signature_.clear();
  active_intersection_.clear();
  show_queue(nullptr);
  health_->setText("Waiting for queue data");
  health_->show();
  try {
    auto inbox = inbox_;
    subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      topic_.trimmed().toStdString(), rclcpp::QoS(1).reliable().transient_local(),
      [inbox](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox->message = msg;
        inbox->received = std::chrono::steady_clock::now();
      });
  } catch (const std::exception & error) {
    health_->setText(QString("Invalid topic: %1").arg(error.what()));
  }
}
void QueuePanel::refresh()
{
  if (!inbox_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message;
  std::chrono::steady_clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    message = inbox_->message;
    received = inbox_->received;
  }
  if (!message) {
    return;
  }
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count();
  if (age > 2.0) {
    health_->setText(QString("QUEUE DATA STALE (%1 s) — last known order").arg(age, 0, 'f', 1));
    health_->show();
    for (auto * slot : slots_) {
      slot->setEnabled(false);
    }
  } else {
    const bool perception_stale = std::any_of(
      message->status.begin(), message->status.end(),
      [](const auto & s) { return s.level != diagnostic_msgs::msg::DiagnosticStatus::OK; });
    health_->setText(
      perception_stale ? "Live queue — object data stale / unavailable" : "Live queue");
    health_->setVisible(perception_stale);
    for (auto * slot : slots_) {
      slot->setEnabled(true);
    }
  }
  // Select only an intersection containing AV. Keep the current selection when
  // polygons overlap so the panel does not jump between intersections.
  const auto contains_av = [](const auto & status) {
    return std::any_of(status.values.begin(), status.values.end(), [](const auto & entry) {
      return entry.key == "AV" && (entry.value == "PRIORITY" || entry.value == "CONFLICT");
    });
  };
  auto active =
    std::find_if(message->status.begin(), message->status.end(), [&](const auto & status) {
      return status.name == active_intersection_ && contains_av(status);
    });
  if (active == message->status.end()) {
    active = std::find_if(message->status.begin(), message->status.end(), contains_av);
  }
  active_intersection_ = active == message->status.end() ? "" : active->name;
  // Ignore timestamps to avoid rebuilding unchanged rows and disturbing scrolling.
  QString signature;
  if (active != message->status.end()) {
    const auto & status = *active;
    signature +=
      QString::fromStdString(status.name + "|" + status.hardware_id + "|" + status.message);
    for (const auto & entry : status.values) {
      signature += QString::fromStdString("|" + entry.key + ":" + entry.value);
    }
    signature += '\n';
  }
  if (active == message->status.end()) {
    signature = "NO_ACTIVE_INTERSECTION";
  }
  if (signature == signature_) {
    return;
  }
  signature_ = signature;
  show_queue(active == message->status.end() ? nullptr : &*active);
}

void QueuePanel::show_queue(const diagnostic_msgs::msg::DiagnosticStatus * status)
{
  title_->setText(
    status ? QString("Intersection %1").arg(QString::fromStdString(status->name))
           : "No active intersection");
  title_->setToolTip(
    status ? QString("VTL ID: %1").arg(QString::fromStdString(status->hardware_id)) : QString());
  position_->clear();
  position_->hide();
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    auto * slot = slots_[i];
    const bool occupied = status && i < status->values.size();
    const bool av = occupied && status->values[i].key == "AV";
    QString label = QString::fromUtf8("—");
    QString tooltip;
    if (occupied) {
      const auto & entry = status->values[i];
      const QString uuid = QString::fromStdString(entry.key);
      label = "AV";
      if (!av) {
        int length = 8;
        while (length < uuid.size() && std::any_of(
                                         status->values.begin(), status->values.end(),
                                         [&](const auto & other) {
                                           return other.key != entry.key &&
                                                  QString::fromStdString(other.key).left(length) ==
                                                    uuid.left(length);
                                         })) {
          ++length;
        }
        label = uuid.left(length);
      }
      tooltip = uuid + " | " + QString::fromStdString(entry.value);
    }
    slot->setText(label);
    slot->setToolTip(tooltip);
    slot->setStyleSheet(QString(
                          "QLabel { background-color: %1; color: #17212b; border: 1px solid %2; "
                          "border-radius: 4px; padding: 6px; font-size: 16px; font-weight: bold; }"
                          "QLabel:disabled { color: #6b7280; background-color: #eeeeee; }")
                          .arg(av ? "#edb80b" : "#8ed1ec")
                          .arg(av ? "#c99a09" : "#65adc9"));
  }
  if (status) {
    const auto ego = std::find_if(
      status->values.begin(), status->values.end(),
      [](const auto & entry) { return entry.key == "AV"; });
    if (ego != status->values.end() && ego - status->values.begin() >= 4) {
      position_->setText(QString("AV position: %1").arg(ego - status->values.begin() + 1));
      position_->show();
    }
  }
}

void QueuePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("QueueTopic", &topic)) {
    topic_ = topic;
    subscribe();
  }
}
void QueuePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("QueueTopic", topic_);
}
}  // namespace autoware::intersection_priority
PLUGINLIB_EXPORT_CLASS(autoware::intersection_priority::QueuePanel, rviz_common::Panel)
