// ============================================================================
// Yolo→Arm→Explore Loop Supervisor (ROS2 Lifecycle, C++)
// ----------------------------------------------------------------------------
// 요구 시나리오(무한 루프):
//  1) 시작 → /yolo_nav2 를 CONFIGURE (필요시) → ACTIVATE
//  2) /yolo_nav2_completed==true 관측 → /yolo_nav2 DEACTIVATE →
//     /robotarm_control CONFIGURE→ACTIVATE
//  3) /robotarm_completed==true 관측 → /explore_node CONFIGURE→ACTIVATE
//  4) /yolo_detect==true 관측 → /explore_node DEACTIVATE→CLEANUP → 1)로 복귀
//  ※ 사용자가 종료할 때까지 반복
//
// 설계 노트:
//  - 각 대상 노드는 ROS2 lifecycle 노드(change_state/get_state 서비스 제공)라고 가정
//  - CONFIGURE는 현재 상태가 UNCONFIGURED일 때만 전송. 이미 configured면 스킵
//  - 이벤트 토픽(yolo_nav2_completed, robotarm_completed, yolo_detect)은 "상태"가 아니라
//    "트리거" 성격이므로 **latched(=transient_local) 사용하지 않음**. true 수신 시
//    즉시 consume(플래그 false로 리셋)하여 중복 트리거를 방지
//  - 서비스 이름은 절대경로("/name/change_state") 사용
//  - 시작 시 GetState로 초기 동기화
//  - 전이 요청은 비동기, 중복 방지, polling으로 결과 반영
//  - 타임아웃/ABORT 처리 포함
//
// 파라미터(ros2 run/launch에서 설정 가능):
//  - yolo_name            (string, default: "yolo_nav2")
//  - arm_name             (string, default: "robotarm_control")
//  - explore_name         (string, default: "explore_node")
//  - topic_yolo_done      (string, default: "/yolo_nav2_completed")
//  - topic_arm_done       (string, default: "/robotarm_completed")
//  - topic_yolo_detect    (string, default: "/yolo_detect")
//  - deactivate_arm_on_done (bool, default: true)  // 3단계 진입 전에 arm 비활성화 권장
//  - tick_ms              (int, default: 100)
//  - timeout_configure_s  (int, default: 20)
//  - timeout_activate_s   (int, default: 15)
//
// 빌드 의존:
//  - rclcpp
//  - lifecycle_msgs
//  - std_msgs
// ----------------------------------------------------------------------------

#include <chrono>
#include <unordered_map>
#include <vector>
#include <string>
#include <future>
#include <optional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState    = lifecycle_msgs::srv::GetState;

// ----------------------------------------------------------------------------
// Data structures
// ----------------------------------------------------------------------------
struct PendingTransition {
  std::string key;          // Managed node key
  uint8_t transition_id;    // Transition id
  rclcpp::Time start_time;  // Dispatch time
  rclcpp::Client<ChangeState>::SharedFuture future; // async response
};

struct ManagedNode {
  std::string key;        // internal key
  std::string name;       // actual node base name
  std::string change_srv; // "/<name>/change_state"
  std::string get_srv;    // "/<name>/get_state"
  rclcpp::Client<ChangeState>::SharedPtr change_cli;
  rclcpp::Client<GetState>::SharedPtr get_cli;

  // cached state flags
  bool configured = false;
  bool active = false;
};

// ----------------------------------------------------------------------------
// Phase enum
// ----------------------------------------------------------------------------
enum class Phase {
  INIT,
  PREPARE_YOLO,       // ensure configured
  ACTIVATE_YOLO,      // activate
  WAIT_YOLO_DONE,     // wait topic_yolo_done
  SWITCH_TO_ARM,      // deactivate YOLO → configure+activate ARM
  WAIT_ARM_DONE,      // wait topic_arm_done
  START_EXPLORE,      // configure+activate EXPLORE
  EXPLORE_RUNNING,    // hold until topic_yolo_detect
  SHUTDOWN_CLEANUP,   // not used in loop, reserved
  ABORT
};

// ----------------------------------------------------------------------------
// Supervisor Node
// ----------------------------------------------------------------------------
class YAE_Supervisor : public rclcpp::Node {
public:
  YAE_Supervisor()
  : Node("yae_lifecycle_supervisor")
  {
    // --- Parameters (declare & get) ---
    yolo_name_     = declare_parameter<std::string>("yolo_name", "yolo_depth_to_nav");
    arm_name_      = declare_parameter<std::string>("arm_name",  "arm_trajectory_client");
    explore_name_  = declare_parameter<std::string>("explore_name", "explore_node");

    topic_yolo_done_   = declare_parameter<std::string>("topic_yolo_done", "/yolo_nav2_completed");
    topic_arm_done_    = declare_parameter<std::string>("topic_arm_done",  "/robotarm_completed");
    topic_yolo_detect_ = declare_parameter<std::string>("topic_yolo_detect","/yolo_detect");

    deactivate_arm_on_done_ = declare_parameter<bool>("deactivate_arm_on_done", true);

    const int tick_ms = declare_parameter<int>("tick_ms", 100);
    timeout_configure_ = rclcpp::Duration(declare_parameter<int>("timeout_configure_s", 20), 0);
    timeout_activate_  = rclcpp::Duration(declare_parameter<int>("timeout_activate_s", 15), 0);

    // --- Register lifecycle nodes ---
    add_node(KEY_YOLO,    yolo_name_);
    add_node(KEY_ARM,     arm_name_);
    add_node(KEY_EXPLORE, explore_name_);

    // --- Subscribe event topics (non-latched; triggers are consumed) ---
    auto qos_event = rclcpp::QoS(rclcpp::KeepLast(10)); // Reliable, volatile

    sub_yolo_done_ = create_subscription<std_msgs::msg::Bool>(
      topic_yolo_done_, qos_event,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){
        if (msg->data) { yolo_done_ = true; RCLCPP_INFO(get_logger(), "[event] yolo_nav2_completed=true"); }
      });

    sub_arm_done_ = create_subscription<std_msgs::msg::Bool>(
      topic_arm_done_, qos_event,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){
        if (msg->data) { arm_done_ = true; RCLCPP_INFO(get_logger(), "[event] robotarm_completed=true"); }
      });

    sub_yolo_detect_ = create_subscription<std_msgs::msg::Bool>(
      topic_yolo_detect_, qos_event,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){
        if (msg->data) { yolo_detect_ = true; RCLCPP_INFO(get_logger(), "[event] yolo_detect=true"); }
      });

    // Initial state sync
    sync_initial_states();

    phase_ = Phase::INIT;
    phase_enter_time_ = now();

    tick_timer_ = create_wall_timer(std::chrono::milliseconds(tick_ms), std::bind(&YAE_Supervisor::tick, this));
    RCLCPP_INFO(get_logger(), "YAE Supervisor initialized (Yolo→Arm→Explore loop)");
  }

private:
  // Keys
  static constexpr const char* KEY_YOLO    = "yolo";
  static constexpr const char* KEY_ARM     = "arm";
  static constexpr const char* KEY_EXPLORE = "explore";

  // --- Node registry ---
  void add_node(const std::string & key, const std::string & name) {
    ManagedNode mn;
    mn.key = key;
    mn.name = name;
    mn.change_srv = "/" + name + "/change_state";
    mn.get_srv    = "/" + name + "/get_state";
    mn.change_cli = create_client<ChangeState>(mn.change_srv);
    mn.get_cli    = create_client<GetState>(mn.get_srv);
    nodes_[key] = std::move(mn);
  }

  // --- Helpers ---
  rclcpp::Time now() { return get_clock()->now(); }
  rclcpp::Duration in_phase() { return now() - phase_enter_time_; }

  void change_phase(Phase next) {
    if (phase_ == next) return;
    RCLCPP_INFO(get_logger(), "PHASE %d -> %d", static_cast<int>(phase_), static_cast<int>(next));
    phase_ = next;
    phase_enter_time_ = now();
  }

  bool is_transition_pending(const std::string & key, uint8_t id) const {
    return std::any_of(pending_.begin(), pending_.end(), [&](auto & p){ return p.key==key && p.transition_id==id; });
  }

  void dispatch_transition(const std::string & key, uint8_t transition_id) {
    auto it = nodes_.find(key);
    if (it == nodes_.end()) return;
    auto & mn = it->second;
    if (is_transition_pending(key, transition_id)) return; // prevent duplicate
    if (!mn.change_cli->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "[%s] change_state service unavailable: %s", key.c_str(), mn.change_srv.c_str());
      return;
    }
    auto req = std::make_shared<ChangeState::Request>();
    req->transition.id = transition_id;
    auto future = mn.change_cli->async_send_request(req);
    pending_.push_back(PendingTransition{ key, transition_id, now(), future });
    RCLCPP_INFO(get_logger(), "[%s] Dispatch transition %u", key.c_str(), transition_id);
  }

  void poll_transitions() {
    auto it = pending_.begin();
    while (it != pending_.end()) {
      if (it->future.wait_for(0ms) == std::future_status::ready) {
        auto resp = it->future.get();
        auto & mn = nodes_.at(it->key);
        bool ok = resp->success;
        RCLCPP_INFO(get_logger(), "[%s] Transition %u result: %s", it->key.c_str(), it->transition_id, ok?"SUCCESS":"FAIL");
        if (ok) {
          apply_local_flags(mn, it->transition_id);
        } else {
          RCLCPP_ERROR(get_logger(), "[%s] Transition %u failed -> ABORT", it->key.c_str(), it->transition_id);
          change_phase(Phase::ABORT);
        }
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void apply_local_flags(ManagedNode & mn, uint8_t transition_id) {
    using lifecycle_msgs::msg::Transition;
    switch (transition_id) {
      case Transition::TRANSITION_CONFIGURE:
        mn.configured = true; mn.active = false; break;
      case Transition::TRANSITION_CLEANUP:
        mn.configured = false; mn.active = false; break;
      case Transition::TRANSITION_ACTIVATE:
        mn.active = true; mn.configured = true; break;
      case Transition::TRANSITION_DEACTIVATE:
        mn.active = false; /* configured stays true */ break;
      default: break;
    }
  }

  void sync_initial_states() {
    for (auto &kv : nodes_) {
      auto &mn = kv.second;
      if (!mn.get_cli->wait_for_service(1s)) continue;
      auto fut = mn.get_cli->async_send_request(std::make_shared<GetState::Request>());
      if (fut.wait_for(500ms) == std::future_status::ready) {
        auto id = fut.get()->current_state.id;
        mn.configured = (id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
                         id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        mn.active = (id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        RCLCPP_INFO(get_logger(), "[%s] initial state: configured=%d active=%d", kv.first.c_str(), (int)mn.configured, (int)mn.active);
      }
    }
  }

  // convenience guards
  void ensure_configured(const std::string & key) {
    auto &mn = nodes_.at(key);
    if (!mn.configured) {
      dispatch_transition(key, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }
  }
  void ensure_active(const std::string & key) {
    auto &mn = nodes_.at(key);
    if (!mn.active) {
      dispatch_transition(key, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }
  }
  void ensure_deactivated(const std::string & key) {
    auto &mn = nodes_.at(key);
    if (mn.active) {
      dispatch_transition(key, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  }
  void ensure_cleanup(const std::string & key) {
    auto &mn = nodes_.at(key);
    if (mn.configured) {
      dispatch_transition(key, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
  }

  bool all_idle() const {
    for (auto &kv : nodes_) if (kv.second.active) return false; return true;
  }

  // --- FSM tick ---
  void tick() {
    poll_transitions();

    switch (phase_) {
      case Phase::INIT: {
        // 바로 YOLO 준비로 진입
        change_phase(Phase::PREPARE_YOLO);
        break; }

      case Phase::PREPARE_YOLO: {
        ensure_configured(KEY_YOLO);
        // timeout for CONFIGURE
        if (nodes_[KEY_YOLO].configured) {
          change_phase(Phase::ACTIVATE_YOLO);
        } else if (in_phase() > timeout_configure_) {
          RCLCPP_ERROR(get_logger(), "YOLO CONFIGURE timeout -> ABORT");
          change_phase(Phase::ABORT);
        }
        break; }

      case Phase::ACTIVATE_YOLO: {
        ensure_active(KEY_YOLO);
        if (nodes_[KEY_YOLO].active) {
          // 새 라운드 시작: 이전 라운드 잔여 이벤트 플래그는 수동 초기화
          yolo_done_ = false; arm_done_ = false; yolo_detect_ = false;
          change_phase(Phase::WAIT_YOLO_DONE);
        } else if (in_phase() > timeout_activate_) {
          RCLCPP_ERROR(get_logger(), "YOLO ACTIVATE timeout -> ABORT");
          change_phase(Phase::ABORT);
        }
        break; }

      case Phase::WAIT_YOLO_DONE: {
        if (yolo_done_) {
          yolo_done_ = false; // consume
          // 1) YOLO deactivate
          ensure_deactivated(KEY_YOLO);
          // 2) switch to ARM
          change_phase(Phase::SWITCH_TO_ARM);
        }
        break; }

      case Phase::SWITCH_TO_ARM: {
        // ensure YOLO really inactive before moving on
        if (!nodes_[KEY_YOLO].active) {
          ensure_configured(KEY_ARM);
          if (nodes_[KEY_ARM].configured) {
            ensure_active(KEY_ARM);
            if (nodes_[KEY_ARM].active) {
              arm_done_ = false; // start waiting fresh
              change_phase(Phase::WAIT_ARM_DONE);
            }
          }
          // timeouts
          if (!nodes_[KEY_ARM].configured && in_phase() > timeout_configure_) {
            RCLCPP_ERROR(get_logger(), "ARM CONFIGURE timeout -> ABORT");
            change_phase(Phase::ABORT);
          }
          if (!nodes_[KEY_ARM].active && in_phase() > timeout_activate_) {
            RCLCPP_ERROR(get_logger(), "ARM ACTIVATE timeout -> ABORT");
            change_phase(Phase::ABORT);
          }
        }
        break; }

      case Phase::WAIT_ARM_DONE: {
        if (arm_done_) {
          arm_done_ = false; // consume
          if (deactivate_arm_on_done_) {
            ensure_deactivated(KEY_ARM);
          }
          change_phase(Phase::START_EXPLORE);
        }
        break; }

      case Phase::START_EXPLORE: {
        // Make sure ARM is deactivated if requested
        if (deactivate_arm_on_done_ && nodes_[KEY_ARM].active) break; // wait until deactivated

        ensure_configured(KEY_EXPLORE);
        if (nodes_[KEY_EXPLORE].configured) {
          ensure_active(KEY_EXPLORE);
          if (nodes_[KEY_EXPLORE].active) {
            yolo_detect_ = false; // fresh wait
            change_phase(Phase::EXPLORE_RUNNING);
          }
        }
        if (!nodes_[KEY_EXPLORE].configured && in_phase() > timeout_configure_) {
          RCLCPP_ERROR(get_logger(), "EXPLORE CONFIGURE timeout -> ABORT");
          change_phase(Phase::ABORT);
        }
        if (!nodes_[KEY_EXPLORE].active && in_phase() > timeout_activate_) {
          RCLCPP_ERROR(get_logger(), "EXPLORE ACTIVATE timeout -> ABORT");
          change_phase(Phase::ABORT);
        }
        break; }

      case Phase::EXPLORE_RUNNING: {
        if (yolo_detect_) {
          yolo_detect_ = false; // consume
          // stop explore → cleanup
          ensure_deactivated(KEY_EXPLORE);
          // wait until inactive then cleanup once
          if (!nodes_[KEY_EXPLORE].active) {
            ensure_cleanup(KEY_EXPLORE);
            // when cleanup confirmed, loop back
            if (!nodes_[KEY_EXPLORE].configured) {
              change_phase(Phase::PREPARE_YOLO);
            }
          }
        }
        break; }

      case Phase::SHUTDOWN_CLEANUP: {
        if (all_idle()) {
          for (auto &kv: nodes_) if (kv.second.configured) ensure_cleanup(kv.first);
        }
        break; }

      case Phase::ABORT: {
        if (!abort_executed_) {
          RCLCPP_ERROR(get_logger(), "ABORT entered → safe deactivation");
          for (auto &kv : nodes_) if (kv.second.active)
            dispatch_transition(kv.first, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          abort_executed_ = true;
        }
        break; }
    }
  }

  // --- Members ---
  Phase phase_;
  rclcpp::Time phase_enter_time_;

  std::unordered_map<std::string, ManagedNode> nodes_;
  std::vector<PendingTransition> pending_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // params
  std::string yolo_name_, arm_name_, explore_name_;
  std::string topic_yolo_done_, topic_arm_done_, topic_yolo_detect_;
  bool deactivate_arm_on_done_ = true;
  rclcpp::Duration timeout_configure_{20,0};
  rclcpp::Duration timeout_activate_{15,0};

  // events (consumed flags)
  bool yolo_done_   = false;
  bool arm_done_    = false;
  bool yolo_detect_ = false;

  // subs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_yolo_done_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_arm_done_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_yolo_detect_;

  bool abort_executed_ = false;
};

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YAE_Supervisor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
