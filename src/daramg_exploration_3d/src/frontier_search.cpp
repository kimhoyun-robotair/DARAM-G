// ───── frontier_octomap_node.cpp (v2 – bound‑safe clustering) ─────
#include <rclcpp/rclcpp.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "frontier_msgs/msg/frontier_info_array.hpp"
#include "frontier_msgs/msg/frontier_info.hpp"
#include <std_msgs/msg/color_rgba.hpp>

#include <array>
#include <queue>
#include <unordered_set>
#include <memory>
#include <chrono>

using octomap::OcTree;
using octomap::ColorOcTree;
using octomap::OcTreeKey;
using octomap::point3d;

/* ─────────────── 자료구조 & 해시 ─────────────── */
struct KeyHash {
  size_t operator()(const OcTreeKey &k) const {
    return (static_cast<size_t>(k.k[0]) << 42) ^
           (static_cast<size_t>(k.k[1]) << 21) ^
            static_cast<size_t>(k.k[2]);
  }
};
struct KeyEqual {
  bool operator()(const OcTreeKey &a, const OcTreeKey &b) const { return a == b; }
};

struct Frontier {
  std::vector<OcTreeKey> cells;      // voxel keys
  geometry_msgs::msg::Point centroid;
};

/* ─────────────── FrontierDetector 노드 ─────────────── */
class FrontierDetector : public rclcpp::Node
{
public:
  FrontierDetector() : Node("frontier_detector")
  {
    this->declare_parameter<bool>("visualize", true);
    this->declare_parameter<double>("min_cluster_m", 0.3);
    this->declare_parameter<std::string>("map_frame", "map");

    this->get_parameter("visualize", visualize_);
    this->get_parameter("min_cluster_m", min_cluster_m_);
    this->get_parameter("map_frame", map_frame_);

    sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", 10,
      std::bind(&FrontierDetector::octomapCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "[Init] Subscribed to /octomap_binary, waiting for data …");

    frontier_info_pub_ = create_publisher<frontier_msgs::msg::FrontierInfoArray>(
                          "frontier_info", 10);

    if (visualize_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "frontier_markers", 10);
    }
  }

private:
  /* ───── helper: UNKNOWN voxel이 FREE voxel에 인접? ───── */
  template<typename TreeT>
  static inline bool isAdjacentToFree(const OcTreeKey &key, TreeT *tree)
  {
    static const int dirs6[6][3] = {
      { 1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
    };
    for (auto d : dirs6) {
      OcTreeKey kn(key[0]+d[0], key[1]+d[1], key[2]+d[2]);
      auto *node = tree->search(kn);
      if (node && !tree->isNodeOccupied(node)) return true;  // FREE neighbour
    }
    return false;
  }

  /* ───────────── 후보 셀을 26-연결 flood-fill (bounded) ───────────── */
  template<typename TreeT>
  static void buildFrontierCluster(
      const OcTreeKey &seed,
      TreeT *tree,
      std::unordered_set<OcTreeKey, KeyHash, KeyEqual> &frontier_flag,
      std::vector<Frontier> &out,
      double min_size_m,
      rclcpp::Logger logger)
  {
    std::queue<OcTreeKey> q;
    std::vector<OcTreeKey> cluster;
    q.push(seed);
    cluster.push_back(seed);

    static const int dirs26[26][3] = {   // 3×3×3 전체 − (0,0,0)
      {-1,-1,-1},{0,-1,-1},{1,-1,-1}, {-1,0,-1},{0,0,-1},{1,0,-1},{-1,1,-1},{0,1,-1},{1,1,-1},
      {-1,-1, 0},{0,-1, 0},{1,-1, 0}, {-1,0, 0},{1,0, 0}, {-1,1, 0},{0,1, 0},{1,1, 0},
      {-1,-1, 1},{0,-1, 1},{1,-1, 1}, {-1,0, 1},{0,0, 1},{1,0, 1},{-1,1, 1},{0,1, 1},{1,1, 1} };

    while (!q.empty()) {
      OcTreeKey k = q.front(); q.pop();
      for (auto d : dirs26) {
        OcTreeKey kn(k[0] + d[0], k[1] + d[1], k[2] + d[2]);

        /* UNKNOWN + 인접 FREE + 아직 표시 안 됨 → 클러스터 확장 */
        if (!tree->search(kn) && frontier_flag.insert(kn).second && isAdjacentToFree(kn, tree)) {
          q.push(kn);
          cluster.push_back(kn);
        }
      }
    }

    // 해상도 × 셀 수 → 실제 길이(m) 필터
    double res = tree->getResolution();
    if (cluster.size() * res < min_size_m) {
      RCLCPP_DEBUG(logger, "[Cluster] Discarded small cluster (%.2f m)", cluster.size()*res);
      return;
    }

    // 클러스터 → Frontier 구조체 변환
    Frontier f;
    point3d centroid_sum(0, 0, 0);
    for (const auto &k : cluster) {
      centroid_sum += tree->keyToCoord(k);
    }
    centroid_sum /= static_cast<float>(cluster.size());

    f.centroid.x = centroid_sum.x();
    f.centroid.y = centroid_sum.y();
    f.centroid.z = centroid_sum.z();
    f.cells = std::move(cluster);
    out.emplace_back(std::move(f));

    RCLCPP_DEBUG(logger, "[Cluster] Frontier #%zu (size=%zu, centroid=%.2f,%.2f,%.2f)",
                 out.size()-1, f.cells.size(), f.centroid.x, f.centroid.y, f.centroid.z);
  }

  /* ───────────── 공통 BFS 로직 (템플릿) ───────────── */
  template<typename TreeT>
  void extractFrontiers(TreeT *tree, const std::string &tree_label)
  {
    auto start = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "[Extract] Begin (%s, res=%.3f m)",
                tree_label.c_str(), tree->getResolution());

    std::unordered_set<OcTreeKey, KeyHash, KeyEqual> visited;
    std::unordered_set<OcTreeKey, KeyHash, KeyEqual> frontier_flag;
    std::vector<Frontier> frontiers;

    std::queue<OcTreeKey> q;
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) continue;
      q.push(it.getKey());
      visited.insert(it.getKey());
    }
    RCLCPP_INFO(this->get_logger(), "[Extract] Initial FREE leafs queued: %zu", visited.size());

    static const std::array<std::array<int, 3>, 6> dirs = {{
        { 1,  0,  0}, {-1,  0,  0}, { 0,  1,  0},
        { 0, -1,  0}, { 0,  0,  1}, { 0,  0, -1}
    }};

    size_t pop_count = 0;
    while (!q.empty()) {
      OcTreeKey k = q.front(); q.pop();
      ++pop_count;

      for (const auto &d : dirs) {
        OcTreeKey kn(k[0] + d[0], k[1] + d[1], k[2] + d[2]);

        if (!tree->search(kn) && isAdjacentToFree(kn, tree)) {  // UNKNOWN + 인접 FREE
          if (frontier_flag.insert(kn).second) {
            buildFrontierCluster(kn, tree, frontier_flag, frontiers, 0.3, get_logger());
          }
          continue;
        }

        auto *n = tree->search(kn);
        if (n && !tree->isNodeOccupied(n) && visited.insert(kn).second) {
          q.push(kn);
        }
      }

      if (pop_count % 5000 == 0) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                          std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(this->get_logger(),
          "[Progress] %zu pops | queue=%zu | visited=%zu | candidates=%zu | frontiers=%zu | %lds",
          pop_count, q.size(), visited.size(), frontier_flag.size(), frontiers.size(), elapsed);
      }
    }

    auto dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - start).count();

    RCLCPP_INFO(this->get_logger(),
        "[Extract] End: visited=%zu, candidates=%zu, frontiers=%zu, time=%ld ms",
        visited.size(), frontier_flag.size(), frontiers.size(), dur_ms);

    double res = tree->getResolution();          // voxel 해상도 (m)

    /* ── 프런티어별 로그 ───────────────────────────── */
    for (size_t i = 0; i < frontiers.size(); ++i) {
      const auto &f = frontiers[i];

      std::size_t n_cells = f.cells.size();
      // (셀 수)^(1/3) × 해상도  → 대략 한 변 길이
      double edge_m = std::cbrt(static_cast<double>(n_cells)) * res;

      RCLCPP_INFO(get_logger(),
          "  [%zu] centroid (%.2f, %.2f, %.2f) | res = %.3f m | cells = %zu | ≈ edge %.2f m",
          i,
          f.centroid.x, f.centroid.y, f.centroid.z,
          res,
          n_cells, edge_m);
    }

    if(visualize_) publishMarkers(frontiers);
    publishFrontierInfo(frontiers, tree->getResolution());
  }

  /* ---------------- Marker publisher ---------------- */
  void publishMarkers(const std::vector<Frontier> &fronts)
  {
    visualization_msgs::msg::MarkerArray arr;
    uint32_t id = 0;

    /* ── 색 정의 ───────────────── */
    std_msgs::msg::ColorRGBA blue;
    blue.r = 0.0f;  blue.g = 0.0f;  blue.b = 1.0f;  blue.a = 1.0f;

    std_msgs::msg::ColorRGBA red;
    red.r = 1.0f;   red.g = 0.0f;   red.b = 0.0f;   red.a = 1.0f;

    std_msgs::msg::ColorRGBA white;
    white.r = 1.0f; white.g = 1.0f; white.b = 1.0f; white.a = 1.0f;

    /* ── 프런티어별 마커 생성 ──── */
    for (std::size_t idx = 0; idx < fronts.size(); ++idx)
    {
      const auto &f = fronts[idx];

      /* ① 중심점 구(SPHERE) */
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = map_frame_;
      sphere.header.stamp    = this->now();
      sphere.ns   = "frontier_centers";
      sphere.id   = id++;                       // 0,2,4,…
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = f.centroid;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15;
      sphere.color = red;
      arr.markers.push_back(std::move(sphere));

      /* ② 인덱스 라벨(TEXT) */
      visualization_msgs::msg::Marker label;
      label.header.frame_id = map_frame_;
      label.header.stamp    = this->now();
      label.ns   = "frontier_labels";
      label.id   = id++;                       // 1,3,5,…
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = f.centroid;
      label.pose.position.z += 0.25;           // 살짝 위로
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.20;                    // 글자 높이(m)
      label.color   = white;
      label.text    = std::to_string(idx);
      arr.markers.push_back(std::move(label));
    }

    /* ── 이전 마커 정리 ─────────── */
    for (uint32_t k = id; k < last_marker_count_; ++k) {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = map_frame_;
      del.header.stamp    = this->now();
      del.ns   = (k % 2 == 0) ? "frontier_centers" : "frontier_labels";
      del.id   = k;
      del.action = visualization_msgs::msg::Marker::DELETE;
      arr.markers.push_back(std::move(del));
    }
    last_marker_count_ = id;

    marker_pub_->publish(arr);
  }


  /* ───────────── Octomap 콜백 ───────────── */
  void octomapCallback(const octomap_msgs::msg::Octomap &msg)
  {
    RCLCPP_DEBUG(get_logger(), "[CB] Octomap msg: %s (binary=%s, bytes=%zu)",
                 msg.id.c_str(), msg.binary ? "true" : "false", msg.data.size());

    std::unique_ptr<octomap::AbstractOcTree> abs(octomap_msgs::binaryMsgToMap(msg));
    if (!abs) {
      RCLCPP_ERROR(get_logger(), "binaryMsgToMap() failed");
      return;
    }
    convKey_ = abs.get(); 

    if (auto *ctree = dynamic_cast<ColorOcTree *>(abs.get())) {
      RCLCPP_INFO(get_logger(), "[CB] ColorOcTree leaf nodes=%zu", ctree->size());
      extractFrontiers(ctree, "ColorOcTree");
    } else if (auto *otree = dynamic_cast<OcTree *>(abs.get())) {
      RCLCPP_INFO(get_logger(), "[CB] OcTree received (leaf nodes=%zu)", otree->size());
      extractFrontiers(otree, "OcTree");
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown Octree type: %s", msg.id.c_str());
    }
  }

  /* -------- Frontier info publisher (FrontierInfoArray) -------- */
  void publishFrontierInfo(const std::vector<Frontier> &frontiers, double res)
  {
    /* ❶ 배열 메시지 */
    frontier_msgs::msg::FrontierInfoArray msg;      // <frontier_msgs/msg/frontier_info_array.hpp>
    msg.header.frame_id = map_frame_;
    msg.header.stamp    = this->now();
    msg.frontiers.reserve(frontiers.size());

    /* ❷ 프런티어별 채우기 */
    for (std::size_t i = 0; i < frontiers.size(); ++i) {
      frontier_msgs::msg::FrontierInfo fi;          // <frontier_msgs/msg/frontier_info.hpp>
      fi.centroid      = frontiers[i].centroid;
      fi.cells         = static_cast<uint32_t>(frontiers[i].cells.size());
      fi.edge_estimate = std::cbrt(static_cast<float>(fi.cells)) * res;
      fi.index         = static_cast<uint32_t>(i);
      msg.frontiers.emplace_back(std::move(fi));
    }

    /* ❸ 퍼블리시 */
    frontier_info_pub_->publish(msg);               // 퍼블리셔 타입은
                                                    // rclcpp::Publisher<
                                                    //   frontier_msgs::msg::FrontierInfoArray>
                                                    // 로 선언‧초기화해야 함
  }



    /* === OcTreeKey ➜ world 좌표 변환 === */
  inline point3d keyToWorld(const OcTreeKey& k) const
  {
    if (auto ot = dynamic_cast<const OcTree*>(convKey_))
      return ot->keyToCoord(k);
    if (auto ct = dynamic_cast<const ColorOcTree*>(convKey_))
      return ct->keyToCoord(k);
    return point3d(0,0,0);               // fallback (convKey_ == nullptr)
  }

  /* ───────────── 멤버 ───────────── */
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<frontier_msgs::msg::FrontierInfoArray>::SharedPtr frontier_info_pub_;
  const octomap::AbstractOcTree* convKey_{nullptr};
  size_t last_marker_count_{0};
  bool visualize_{true};
  double min_cluster_m_{0.3};
  std::string map_frame_="map";
};

/* ─────────────── main ─────────────── */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierDetector>());
  rclcpp::shutdown();
  return 0;
}
