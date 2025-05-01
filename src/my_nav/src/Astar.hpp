#include <math.h>

#include <algorithm>
#include <queue>
#include <vector>

namespace astar {

/**
 * @brief 定义一个结构体来表示一个点坐标，定义了一个有参构造函数，和默认构造
 * @param row 第row行，从零开始数
 * @param col 第col列，从零开始数
 */
struct Point {
  // 行
  int row;
  // 列
  int col;
  Point(int _row, int _col) {
    row = _row;
    col = _col;
  }
  Point(){};
};

/**
 * @brief 定义一个结构体来表示栅格地图的节点,含有一个构造函数
 * @param point 节点坐标，表示第row行，第col列，从零开始数
 * @param g 从从起始节点到当前节点的代价
 * @param h  从当前节点到目标节点的启发式代价
 * @param f 总代价 (g + h)
 * @param parent 指向父节点的指针
 *
 */
struct Node {
  Point point;
  double g;
  double h;
  double f;
  Node *parent;
  Node(Point _point, int _g, int _h, Node *_parent)
      : point(_point), g(_g), h(_h), parent(_parent) {
    f = g + h;
  }
};

/**
 * @brief 定义一个比较函数来表示开放优先列表的优先级，f更小的优先级更高
 */
struct CompareF {
  bool operator()(const Node *a, const Node *b) const { return a->f > b->f; }
};

class Astar {
 private:
  // 一个向量来存储可能的移动
  const std::vector<std::pair<int, int>> moves_ = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, 1}, {1, -1}};
  // 一个二维向量来存储网格地图
  std::vector<std::vector<int>> grid_;
  // 一个二维向量来存储已访问过的节点
  std::vector<std::vector<bool>> visited_;
  // 一个优先队列来存储开放列表中的节点 (按 f 值排序)
  std::priority_queue<astar::Node *, std::vector<astar::Node *>, CompareF>
      open_;
  int grid_row_;
  int grid_col_;

 public:
  Astar() {}
  /**
   * @brief 计算两个点之间的曼哈顿距离
   * @param _p1 点
   * @param _p2 点
   * @return 曼哈顿距离，两个点横向和纵向距离差的绝对值之和
   */
  double heuristic(Point _p1, Point _p2) {
    return abs(_p1.row-_p2.row)+abs(_p1.col - _p2.col);
  }

  void setMapGrid(std::vector<std::vector<int>> _grid) {
    this->grid_ = _grid;
    this->grid_row_ = this->grid_.size();
    this->grid_col_ = this->grid_[0].size();
    // 初始化访问向量，用 false 值填充
    this->visited_.resize(this->grid_.size(), std::vector<bool>(this->grid_[0].size(), false));
  }
  /**
   * @brief 检查一个点是否有效 (在网格内且不是墙)
   * @param _p 一个节点的point属性
   * @return bool值
   */
  bool isValid(Point _p) {
    return _p.col >= 0 && _p.col < this->grid_col_ && _p.row >= 0 &&
           _p.row < this->grid_row_ && this->grid_[_p.row][_p.col] == 0;
  }

  /**
   * @brief 使用 A* 算法找到从起始节点到目标节点的最短路径
   * @param _startp 起点的point属性
   * @param _goalp 目标点的point属性
   * @return 得到的路径点集合
   */
  std::vector<astar::Point> findPath(Point _startp, Point _goalp) {
    if (!this->isValid(_startp) || !isValid(_goalp)) {
      return {};
    }
    // 重置visited_
    for (auto &v : this->visited_) {
      // 将每个 std::vector<bool> 都填充为 false
      std::fill(v.begin(), v.end(), false);
    }
    // 清空open
    while (!this->open_.empty()) {
      this->open_.pop();
    }
    // 创建一个起始节点和一个目标节点
    Node *start_node =
        new Node(_startp, 0, heuristic(_startp, _goalp), nullptr);
    this->open_.push(start_node);
    this->visited_[start_node->point.row][start_node->point.col] = true;
    while (!this->open_.empty()) {
      Node *current = this->open_.top();
      this->open_.pop();
      // 检查当前节点是否是目标节点
      if (current->point.row == _goalp.row &&
          current->point.col == _goalp.col) {
        // 找到目标节点，通过回溯父节点构造路径
        std::vector<astar::Point> path;
        while (current != nullptr) {
          path.push_back(current->point);
          current = current->parent;
        }
        // 反转路径并返回
        std::reverse(path.begin(), path.end());
        //
        return path;
      }
      // 遍历当前节点可能的移动
      for (auto move : this->moves_) {
        // 通过加上移动偏移量得到下一个节点的坐标
        astar::Point next_point;
        next_point.row = current->point.row + move.first;
        next_point.col = current->point.col + move.second;
        if (!isValid(next_point) || visited_[next_point.row][next_point.col]) {
          continue;
        }
        if (abs(move.first) + abs(move.second) == 2) {
          if (this->grid_[current->point.row + move.first]
                         [current->point.col] == 1 &&
              this->grid_[current->point.row]
                         [current->point.col + move.second] == 1) {
            continue;
          }
        }
        // 创建一个新的节点，更新代价和父指针
        Node *next_node = new Node(
            next_point,
            current->g + std::sqrt(pow(move.first, 2) + pow(move.second, 2)),
            // current->g + 1,
            heuristic(next_point, _goalp), current);
        // 将下一个节点加入开放列表
        this->open_.push(next_node);
        // 将下一个节点标记为已访问
        this->visited_[next_node->point.row][next_node->point.col] = true;
      }
    }
    return {};
  }
  ~Astar() {}
};
}  // namespace astar
