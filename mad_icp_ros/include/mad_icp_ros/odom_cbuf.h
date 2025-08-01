#include <Eigen/Dense>

namespace mad_icp_ros {
// A buffer for wheel odometry messages that allows for easy interpolation
class OdomCircularBuffer {
 public:
  using PoseT = Eigen::Isometry3d;
  using TimeT = double;

  using ItemT = std::pair<TimeT, PoseT>;

  void insert(const ItemT& item);

  ItemT query(TimeT t) const;

  bool is_valid(const ItemT& item) const { return !std::isnan(item.first); }

 protected:
  static constexpr size_t buffer_size_{128};
  std::array<ItemT, buffer_size_> data_;
  size_t start_{0};
  size_t end_{0};
  bool full_{false};

  inline size_t wrap(size_t idx) const { return idx % buffer_size_; }
  inline size_t incr(size_t idx) const { return wrap(idx + 1); }
  inline size_t decr(size_t idx) const { return wrap(idx - 1); }
  inline size_t numel() const {
    return full_ ? buffer_size_
                 : (end_ >= start_ ? end_ - start_
                                   : buffer_size_ - start_ + end_);
  }

  ItemT invalid_item_{std::numeric_limits<TimeT>::quiet_NaN(),
                      PoseT::Identity()};

  size_t find(TimeT t) const;

  PoseT interp(const PoseT& A, const PoseT& B, double lambda) const;
};

}  // namespace mad_icp_ros