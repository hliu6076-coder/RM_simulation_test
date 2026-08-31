#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/World.hh>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rm_combat_interfaces/msg/authorized_shot.hpp>
#include <rm_combat_interfaces/msg/shot_result.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rm_combat_gazebo
{

class LogicalShooterPlugin final : public gazebo::ModelPlugin
{
public:
  LogicalShooterPlugin() = default;

  ~LogicalShooterPlugin() override
  {
    running_.store(false);
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    update_connection_.reset();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    robot_id_ = readSdf<std::string>(sdf, "robot_id", model_->GetName());
    opponent_id_ = readSdf<std::string>(sdf, "opponent_id", "blue_target");
    authorized_topic_ =
      readSdf<std::string>(sdf, "authorized_topic", "/referee/internal/authorized_shots");
    result_topic_ = readSdf<std::string>(sdf, "result_topic", "/referee/shot_results");
    marker_topic_ = readSdf<std::string>(sdf, "marker_topic", "/combat/tracers");
    muzzle_offset_.X(readSdf<double>(sdf, "muzzle_x", 0.25));
    muzzle_offset_.Y(readSdf<double>(sdf, "muzzle_y", 0.0));
    muzzle_offset_.Z(readSdf<double>(sdf, "muzzle_z", 0.18));
    projectile_speed_ = std::max(0.1, readSdf<double>(sdf, "projectile_speed", 12.0));
    projectile_radius_ = std::max(0.001, readSdf<double>(sdf, "projectile_radius", 0.02));

    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    const auto node_name = std::string("logical_shooter_") + sanitize(robot_id_);
    node_ = std::make_shared<rclcpp::Node>(node_name);
    result_pub_ = node_->create_publisher<rm_combat_interfaces::msg::ShotResult>(
      result_topic_, rclcpp::QoS(20).reliable());
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(20).reliable());
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/referee/internal/model_pose", rclcpp::QoS(5).reliable());
    shot_sub_ = node_->create_subscription<rm_combat_interfaces::msg::AuthorizedShot>(
      authorized_topic_, rclcpp::QoS(20).reliable(),
      [this](rm_combat_interfaces::msg::AuthorizedShot::ConstSharedPtr msg) {
        if (msg->shooter_id != robot_id_) {
          return;
        }
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_shots_.push(*msg);
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_.store(true);
    spin_thread_ = std::thread([this]() {
      while (running_.load() && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(20));
      }
    });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&LogicalShooterPlugin::onUpdate, this));
    RCLCPP_INFO(node_->get_logger(), "logical shooter ready for '%s'", robot_id_.c_str());
  }

private:
  template <typename T>
  static T readSdf(const sdf::ElementPtr & sdf, const std::string & key, const T & fallback)
  {
    return sdf && sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
  }

  static std::string sanitize(std::string value)
  {
    std::replace_if(
      value.begin(), value.end(),
      [](char c) { return !(std::isalnum(static_cast<unsigned char>(c)) || c == '_'); }, '_');
    return value;
  }

  void onUpdate()
  {
    publishGroundTruthPose();
    std::queue<rm_combat_interfaces::msg::AuthorizedShot> local;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      std::swap(local, pending_shots_);
    }
    while (!local.empty()) {
      castShot(local.front());
      local.pop();
    }
    updateProjectiles();
  }

  void publishGroundTruthPose()
  {
    const double sim_time = world_->SimTime().Double();
    if (sim_time - last_pose_publish_time_ < 0.1) {
      return;
    }
    last_pose_publish_time_ = sim_time;
    const auto pose = model_->WorldPose();
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = node_->now();
    message.header.frame_id = "world";
    message.pose.position.x = pose.Pos().X();
    message.pose.position.y = pose.Pos().Y();
    message.pose.position.z = pose.Pos().Z();
    message.pose.orientation.x = pose.Rot().X();
    message.pose.orientation.y = pose.Rot().Y();
    message.pose.orientation.z = pose.Rot().Z();
    message.pose.orientation.w = pose.Rot().W();
    pose_pub_->publish(message);
  }

  void castShot(const rm_combat_interfaces::msg::AuthorizedShot & shot)
  {
    auto physics = world_->Physics();
    physics->InitForThread();
    auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      physics->CreateShape("ray", gazebo::physics::CollisionPtr()));
    if (!ray) {
      RCLCPP_ERROR(node_->get_logger(), "failed to create ray shape");
      return;
    }
    const auto pose = model_->WorldPose();
    const auto origin = pose.Pos() + pose.Rot().RotateVector(muzzle_offset_);
    const double cp = std::cos(shot.pitch);
    ignition::math::Vector3d local_direction(
      cp * std::cos(shot.yaw), cp * std::sin(shot.yaw), std::sin(shot.pitch));
    local_direction.Normalize();
    const auto direction = pose.Rot().RotateVector(local_direction);
    const double max_range = std::max(0.01, shot.max_range);
    const auto max_endpoint = origin + direction * max_range;

    ray->Reset();
    ray->SetPoints(origin, max_endpoint);
    double distance = max_range;
    std::string collision_name;
    ray->GetIntersection(distance, collision_name);
    const bool collided = !collision_name.empty() && distance <= max_range;
    const auto endpoint = collided ? origin + direction * distance : max_endpoint;

    rm_combat_interfaces::msg::ShotResult result;
    result.header.stamp = node_->now();
    result.header.frame_id = "world";
    result.shot_id = shot.shot_id;
    result.shooter_id = shot.shooter_id;
    result.collision_name = collision_name;
    result.distance = collided ? distance : max_range;
    setPoint(result.origin, origin);
    setPoint(result.endpoint, endpoint);

    const std::string target_id = collisionModelName(collision_name);
    if (collided && !opponent_id_.empty() && target_id == opponent_id_) {
      result.outcome = rm_combat_interfaces::msg::ShotResult::HIT;
      result.target_id = target_id;
      result.armor_id = "body";
    } else if (collided) {
      result.outcome = rm_combat_interfaces::msg::ShotResult::BLOCKED;
    } else {
      result.outcome = rm_combat_interfaces::msg::ShotResult::MISS;
    }
    ActiveProjectile projectile;
    projectile.shot_id = shot.shot_id;
    projectile.model_name = "combat_projectile_" + sanitize(robot_id_) + "_" +
                            std::to_string(shot.shot_id) + "_" +
                            std::to_string(projectile_serial_++);
    projectile.origin = origin;
    projectile.direction = direction;
    projectile.endpoint = endpoint;
    projectile.travel_distance = result.distance;
    projectile.start_time = world_->SimTime().Double();
    projectile.result = result;
    world_->InsertModelString(makeProjectileSdf(projectile));
    active_projectiles_.push_back(std::move(projectile));
    if (shot.shot_id == 1) {
      RCLCPP_INFO(
        node_->get_logger(),
        "diagnostic shot: model=(%.3f, %.3f, %.3f), ray=(%.3f, %.3f, %.3f)->"
        "(%.3f, %.3f, %.3f), collision='%s'",
        pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), origin.X(), origin.Y(), origin.Z(),
        endpoint.X(), endpoint.Y(), endpoint.Z(), collision_name.c_str());
    }
  }

  struct ActiveProjectile
  {
    uint64_t shot_id{0};
    std::string model_name;
    ignition::math::Vector3d origin;
    ignition::math::Vector3d direction;
    ignition::math::Vector3d endpoint;
    double travel_distance{0.0};
    double start_time{0.0};
    gazebo::physics::ModelPtr model;
    rm_combat_interfaces::msg::ShotResult result;
  };

  std::string makeProjectileSdf(const ActiveProjectile & projectile) const
  {
    std::ostringstream xml;
    xml << "<sdf version='1.6'><model name='" << projectile.model_name << "'>"
        << "<static>true</static><pose>" << projectile.origin.X() << ' ' << projectile.origin.Y()
        << ' ' << projectile.origin.Z() << " 0 0 0</pose>"
        << "<link name='projectile_link'><gravity>false</gravity>"
        << "<visual name='projectile_visual'><geometry><sphere><radius>" << projectile_radius_
        << "</radius></sphere></geometry><material>"
        << "<ambient>1 0.35 0 1</ambient><diffuse>1 0.35 0 1</diffuse>"
        << "<emissive>0.6 0.12 0 1</emissive></material></visual>"
        << "</link></model></sdf>";
    return xml.str();
  }

  void updateProjectiles()
  {
    const double now = world_->SimTime().Double();
    auto projectile = active_projectiles_.begin();
    while (projectile != active_projectiles_.end()) {
      if (!projectile->model) {
        projectile->model = world_->ModelByName(projectile->model_name);
      }

      const double elapsed = std::max(0.0, now - projectile->start_time);
      const double distance = std::min(projectile->travel_distance, projectile_speed_ * elapsed);
      const auto position = projectile->origin + projectile->direction * distance;

      if (projectile->model) {
        projectile->model->SetWorldPose(
          ignition::math::Pose3d(position.X(), position.Y(), position.Z(), 0.0, 0.0, 0.0));
      }
      publishProjectileMarker(*projectile, position);

      const bool arrived = distance >= projectile->travel_distance;
      const bool insertion_failed = !projectile->model && elapsed > 1.0;
      if (!arrived && !insertion_failed) {
        ++projectile;
        continue;
      }

      projectile->result.header.stamp = node_->now();
      result_pub_->publish(projectile->result);
      publishMarker(projectile->result);
      if (projectile->model) {
        world_->RemoveModel(projectile->model);
      }
      if (insertion_failed) {
        RCLCPP_WARN(
          node_->get_logger(), "projectile model '%s' was not inserted; result still published",
          projectile->model_name.c_str());
      }
      projectile = active_projectiles_.erase(projectile);
    }
  }

  void publishProjectileMarker(
    const ActiveProjectile & projectile, const ignition::math::Vector3d & position)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = node_->now();
    marker.header.frame_id = "world";
    marker.ns = "projectile_balls";
    marker.id = static_cast<int32_t>(projectile.shot_id % std::numeric_limits<int32_t>::max());
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.X();
    marker.pose.position.y = position.Y();
    marker.pose.position.z = position.Z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = projectile_radius_ * 2.0;
    marker.scale.y = projectile_radius_ * 2.0;
    marker.scale.z = projectile_radius_ * 2.0;
    marker.color.r = 1.0F;
    marker.color.g = 0.35F;
    marker.color.a = 1.0F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.15);
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(marker);
    marker_pub_->publish(array);
  }

  static void setPoint(geometry_msgs::msg::Point & output, const ignition::math::Vector3d & input)
  {
    output.x = input.X();
    output.y = input.Y();
    output.z = input.Z();
  }

  static std::string collisionModelName(const std::string & collision)
  {
    const auto model_end = collision.find("::");
    return model_end == std::string::npos ? std::string() : collision.substr(0, model_end);
  }

  void publishMarker(const rm_combat_interfaces::msg::ShotResult & result)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = result.header;
    marker.ns = "logical_projectiles";
    marker.id = static_cast<int32_t>(result.shot_id % std::numeric_limits<int32_t>::max());
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.025;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.points = {result.origin, result.endpoint};
    marker.color.a = 1.0F;
    if (result.outcome == rm_combat_interfaces::msg::ShotResult::HIT) {
      marker.color.g = 1.0F;
    } else if (result.outcome == rm_combat_interfaces::msg::ShotResult::BLOCKED) {
      marker.color.r = 1.0F;
      marker.color.g = 0.8F;
    } else {
      marker.color.r = 1.0F;
    }
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(marker);
    marker_pub_->publish(array);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  ignition::math::Vector3d muzzle_offset_;
  std::string robot_id_;
  std::string opponent_id_;
  std::string authorized_topic_;
  std::string result_topic_;
  std::string marker_topic_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rm_combat_interfaces::msg::AuthorizedShot>::SharedPtr shot_sub_;
  rclcpp::Publisher<rm_combat_interfaces::msg::ShotResult>::SharedPtr result_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
  std::mutex queue_mutex_;
  std::queue<rm_combat_interfaces::msg::AuthorizedShot> pending_shots_;
  std::vector<ActiveProjectile> active_projectiles_;
  uint64_t projectile_serial_{0};
  double projectile_speed_{12.0};
  double projectile_radius_{0.02};
  double last_pose_publish_time_{-1.0};
};

GZ_REGISTER_MODEL_PLUGIN(LogicalShooterPlugin)

}  // namespace rm_combat_gazebo
