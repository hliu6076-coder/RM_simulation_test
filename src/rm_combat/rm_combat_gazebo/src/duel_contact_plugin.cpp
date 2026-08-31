#include <algorithm>
#include <cmath>
#include <cstdint>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <limits>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rm_combat_interfaces/msg/collision_report.hpp>
#include <set>
#include <string>
#include <utility>

namespace rm_combat_gazebo
{

class DuelContactPlugin final : public gazebo::WorldPlugin
{
public:
  DuelContactPlugin() = default;

  ~DuelContactPlugin() override
  {
    update_connection_.reset();
    if (contact_manager_ && !never_drop_contacts_before_) {
      contact_manager_->SetNeverDropContacts(false);
    }
  }

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = std::move(world);
    red_model_ = readSdf<std::string>(sdf, "red_model", "red_robot");
    blue_model_ = readSdf<std::string>(sdf, "blue_model", "blue_robot");
    topic_ = readSdf<std::string>(sdf, "report_topic", "/referee/internal/collision_reports");
    update_rate_ = std::max(1.0, readSdf<double>(sdf, "update_rate", 50.0));
    min_speed_ = std::max(0.0, readSdf<double>(sdf, "min_speed", 0.5));
    separation_time_ = std::max(0.0, readSdf<double>(sdf, "separation_time", 0.3));
    cooldown_ = std::max(0.0, readSdf<double>(sdf, "cooldown", 1.0));
    impact_memory_ = std::max(0.0, readSdf<double>(sdf, "impact_memory", 0.1));
    horizontal_normal_z_ = std::clamp(readSdf<double>(sdf, "horizontal_normal_z", 0.7), 0.0, 1.0);

    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("duel_contact_monitor");
    report_pub_ = node_->create_publisher<rm_combat_interfaces::msg::CollisionReport>(
      topic_, rclcpp::QoS(20).reliable());

    contact_manager_ = world_->Physics()->GetContactManager();
    never_drop_contacts_before_ = contact_manager_->NeverDropContacts();
    contact_manager_->SetNeverDropContacts(true);
    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&DuelContactPlugin::onUpdate, this));
    RCLCPP_INFO(
      node_->get_logger(),
      "duel contact monitor ready: red=%s blue=%s rate=%.1fHz threshold=%.2fm/s",
      red_model_.c_str(), blue_model_.c_str(), update_rate_, min_speed_);
  }

private:
  struct Observation
  {
    std::string model_a;
    std::string model_b;
    std::string collision_a;
    std::string collision_b;
    double relative_speed{0.0};
  };

  struct Episode
  {
    bool active{false};
    bool damaged{false};
    double last_seen{-std::numeric_limits<double>::infinity()};
    double last_damage{-std::numeric_limits<double>::infinity()};
  };

  struct RecentSpeed
  {
    double value{0.0};
    double stamp{-std::numeric_limits<double>::infinity()};
  };

  void rememberSpeed(RecentSpeed & recent, double speed, double now)
  {
    if (speed >= recent.value || now - recent.stamp > impact_memory_) {
      recent.value = speed;
      recent.stamp = now;
    }
  }

  double recentSpeed(const RecentSpeed & recent, double now) const
  {
    return now - recent.stamp <= impact_memory_ ? recent.value : 0.0;
  }

  template <typename T>
  static T readSdf(const sdf::ElementPtr & sdf, const std::string & key, const T & fallback)
  {
    return sdf && sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
  }

  bool isParticipant(const std::string & model_name) const
  {
    return model_name == red_model_ || model_name == blue_model_;
  }

  static bool ignoredModel(const std::string & model_name)
  {
    return model_name == "ground_plane" || model_name.find("spawn_zone") != std::string::npos ||
           model_name.rfind("combat_projectile_", 0) == 0;
  }

  bool hasHorizontalContact(const gazebo::physics::Contact & contact) const
  {
    for (int index = 0; index < contact.count; ++index) {
      if (std::abs(contact.normals[index].Z()) < horizontal_normal_z_) {
        return true;
      }
    }
    return false;
  }

  static std::string pairKey(const std::string & a, const std::string & b)
  {
    return a < b ? a + "\n" + b : b + "\n" + a;
  }

  void onUpdate()
  {
    const double now = world_->SimTime().Double();
    if (now < last_update_time_) {
      episodes_.clear();
      recent_model_speeds_.clear();
      recent_robot_relative_speed_ = RecentSpeed{};
      last_update_time_ = -std::numeric_limits<double>::infinity();
    }

    // ODE may remove the closing velocity just before a multi-link wheel
    // contact becomes visible. Remember only the recent participant velocity
    // while continuing to inspect ContactManager at <= 50 Hz.
    const auto red = world_->ModelByName(red_model_);
    const auto blue = world_->ModelByName(blue_model_);
    if (red) {
      rememberSpeed(recent_model_speeds_[red_model_], red->WorldLinearVel().Length(), now);
    }
    if (blue) {
      rememberSpeed(recent_model_speeds_[blue_model_], blue->WorldLinearVel().Length(), now);
    }
    if (red && blue) {
      rememberSpeed(
        recent_robot_relative_speed_, (red->WorldLinearVel() - blue->WorldLinearVel()).Length(),
        now);
    }
    if (now - last_update_time_ + 1e-9 < 1.0 / update_rate_) {
      return;
    }
    last_update_time_ = now;

    std::map<std::string, Observation> observations;
    const auto contact_count = contact_manager_->GetContactCount();
    for (unsigned int index = 0; index < contact_count; ++index) {
      const auto * contact = contact_manager_->GetContact(index);
      if (!contact || !contact->collision1 || !contact->collision2) {
        continue;
      }
      const auto link_a = contact->collision1->GetLink();
      const auto link_b = contact->collision2->GetLink();
      if (!link_a || !link_b || !link_a->GetModel() || !link_b->GetModel()) {
        continue;
      }
      const auto model_a_ptr = link_a->GetModel();
      const auto model_b_ptr = link_b->GetModel();
      const std::string model_a = model_a_ptr->GetName();
      const std::string model_b = model_b_ptr->GetName();
      const bool a_participant = isParticipant(model_a);
      const bool b_participant = isParticipant(model_b);
      if ((!a_participant && !b_participant) || model_a == model_b) {
        continue;
      }
      if ((!a_participant && ignoredModel(model_a)) || (!b_participant && ignoredModel(model_b))) {
        continue;
      }
      const bool robot_collision = a_participant && b_participant;
      if (!robot_collision && !hasHorizontalContact(*contact)) {
        continue;
      }
      double relative_speed =
        (model_a_ptr->WorldLinearVel() - model_b_ptr->WorldLinearVel()).Length();
      if (robot_collision) {
        relative_speed = std::max(relative_speed, recentSpeed(recent_robot_relative_speed_, now));
      } else if (a_participant) {
        relative_speed = std::max(relative_speed, recentSpeed(recent_model_speeds_[model_a], now));
      } else if (b_participant) {
        relative_speed = std::max(relative_speed, recentSpeed(recent_model_speeds_[model_b], now));
      }
      const std::string key = pairKey(model_a, model_b);
      auto & observation = observations[key];
      if (relative_speed >= observation.relative_speed) {
        observation.model_a = model_a;
        observation.model_b = model_b;
        observation.collision_a = contact->collision1->GetScopedName();
        observation.collision_b = contact->collision2->GetScopedName();
        observation.relative_speed = relative_speed;
      }
    }

    std::set<std::string> seen;
    for (const auto & item : observations) {
      const auto & key = item.first;
      const auto & observation = item.second;
      seen.insert(key);
      auto & episode = episodes_[key];
      episode.last_seen = now;
      if (
        !episode.damaged && observation.relative_speed >= min_speed_ &&
        now - episode.last_damage + 1e-9 >= cooldown_) {
        publishReport(observation, now);
        episode.last_damage = now;
        episode.damaged = true;
      }
      episode.active = true;
    }

    for (auto iterator = episodes_.begin(); iterator != episodes_.end();) {
      if (
        seen.find(iterator->first) == seen.end() &&
        now - iterator->second.last_seen >= separation_time_) {
        iterator->second.active = false;
        iterator->second.damaged = false;
      }
      if (!iterator->second.active && now - iterator->second.last_seen > 30.0) {
        iterator = episodes_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void publishReport(const Observation & observation, double sim_time)
  {
    rm_combat_interfaces::msg::CollisionReport report;
    const int64_t sim_nanoseconds = static_cast<int64_t>(sim_time * 1e9);
    report.header.stamp.sec = static_cast<int32_t>(sim_nanoseconds / 1000000000LL);
    report.header.stamp.nanosec = static_cast<uint32_t>(sim_nanoseconds % 1000000000LL);
    report.header.frame_id = "world";
    report.event_id = next_event_id_++;
    report.model_a = observation.model_a;
    report.model_b = observation.model_b;
    report.collision_a = observation.collision_a;
    report.collision_b = observation.collision_b;
    report.relative_speed = observation.relative_speed;
    report_pub_->publish(report);
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ContactManager * contact_manager_{nullptr};
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rm_combat_interfaces::msg::CollisionReport>::SharedPtr report_pub_;
  std::map<std::string, Episode> episodes_;
  std::map<std::string, RecentSpeed> recent_model_speeds_;
  std::string red_model_;
  std::string blue_model_;
  std::string topic_;
  double update_rate_{50.0};
  double min_speed_{0.5};
  double separation_time_{0.3};
  double cooldown_{1.0};
  double impact_memory_{0.1};
  double horizontal_normal_z_{0.7};
  double last_update_time_{-std::numeric_limits<double>::infinity()};
  RecentSpeed recent_robot_relative_speed_;
  uint64_t next_event_id_{1};
  bool never_drop_contacts_before_{false};
};

GZ_REGISTER_WORLD_PLUGIN(DuelContactPlugin)

}  // namespace rm_combat_gazebo
