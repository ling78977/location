#include "Astar.hpp"
// #include <iostream>
#include <stack>
#include <unordered_set>
#include <vector>
class CCPPPlaner {
private:
  /* data */
  std::vector<std::vector<int>> map_;
  std::vector<std::vector<int>> B_;
  std::vector<std::vector<int>> A_;
  astar::Astar astar_planer_;
  int node_num_ = 0;
  int map_row_;
  int map_col_;
  void graphConvert() {
    if (!this->B_.empty()) {
      this->B_.clear();
    }
    int map_rows = this->map_.size();    // Number of rows
    int map_cols = this->map_[0].size(); // Number of columns
    // Horizontal connections

    for (int i = 0; i < map_rows; ++i) {
      std::vector<int> index;
      for (int j = 0; j < map_cols; ++j) {
        if (this->map_[i][j] == 0) {
          index.push_back(j);
        }
      }
      if (index.empty()) {
        continue;
      }
      for (size_t j = 0; j < index.size() - 1; ++j) {
        if (index[j + 1] - index[j] == 1) {
          this->B_.push_back(
              {index[j] + i * map_cols, index[j + 1] + i * map_cols});
        }
      }
    }

    // Vertical connections
    for (int i = 0; i < map_cols; ++i) {
      std::vector<int> index;
      for (int j = 0; j < map_rows; ++j) {
        if (this->map_[j][i] == 0) {
          index.push_back(j);
        }
      }
      if (index.empty()) {
        continue;
      }
      for (size_t j = 0; j < index.size() - 1; ++j) {
        if (index[j + 1] - index[j] == 1) {
          this->B_.push_back(
              {i + index[j] * map_cols, i + index[j + 1] * map_cols});
        }
      }
    }

    // Sort the results based on the first element
    std::sort(this->B_.begin(), this->B_.end(),
              [](const std::vector<int> &a, const std::vector<int> &b) {
                return a[0] < b[0];
              });
  }

  void compresStable2Matrix() {
    // 初始化最大值为最小整数
    int maxVal = std::numeric_limits<int>::min();

    // 遍历二维 vector 找到最大值
    for (const auto &row : this->B_) {
      for (int num : row) {
        if (num > maxVal) {
          maxVal = num;
        }
      }
    }
    if (!this->A_.empty()) {
      this->A_.clear();
    }

    this->A_.resize(maxVal + 1, std::vector<int>(maxVal + 1, 0));
    for (const auto &row : this->B_) {
      this->A_[row[0]][row[1]] = 1; // 设置 A[i, j] 为 1
      this->A_[row[1]][row[0]] = 1; // 设置 A[j, i] 为 1
    }
  }

  astar::Point num2RowCol(int num) {
    return astar::Point(num / this->map_col_, num % this->map_col_);
  }

  std::vector<astar::Point> completePath(std::vector<astar::Point> re) {
    std::vector<astar::Point> results;
    for (int i = 1; i < (int)re.size(); i++) {
      results.emplace_back(re[i - 1]);
      if (abs(re[i].row - re[i - 1].row) + abs(re[i].col - re[i - 1].col) >=
          2) {

        astar::Point startp = re[i - 1];
        astar::Point goalp = re[i];

        std::vector<astar::Point> path =
            this->astar_planer_.findPath(startp, goalp);

        // std::cout << "path size: " << path.size() << std::endl;
        for (int j = 1; j < (int)path.size() - 1; j++) {
          results.emplace_back(path[j]);
        }
      }
      if (i == (int)re.size() - 1) {
        results.emplace_back(re[i]);
      }
    }
    return results;
  }

public:
  CCPPPlaner(std::vector<std::vector<int>> map) { this->updateMap(map); }
  CCPPPlaner() {}
  ~CCPPPlaner() {}
  void updateMap(std::vector<std::vector<int>> map) {
    this->map_ = map;
    astar_planer_.setMapGrid(map);
    // 更新B
    this->graphConvert();
    // 更新A
    this->compresStable2Matrix();
    // 遍历每一行
    this->node_num_ = 0;
    for (const auto &row : this->A_) {
      // 检查当前行是否有元素等于 1
      if (std::any_of(row.begin(), row.end(), [](int x) { return x == 1; })) {
        this->node_num_++;
      }
    }

    this->map_row_ = this->map_.size();
    this->map_col_ = this->map_[0].size();
  }
  bool isMapEmpty() { return this->map_.empty(); }
  std::vector<astar::Point> DFSFindPath() {
    std::stack<int> stack;
    std::unordered_set<int> flag;
    // std::vector<std::pair<int, int>> re;
    std::vector<astar::Point> re;
    int count = 1;
    // 从节点 0 开始
    for (int iter = 0; iter < node_num_; iter++) {
      if ((map_[this->num2RowCol(iter).row][this->num2RowCol(iter).col]) == 0) {
        stack.push(iter);
        flag.insert(iter);
        re.emplace_back(astar::Point(this->num2RowCol(iter)));
        break;
      }
    }
    while (!stack.empty()) {
      int pre_len = stack.size();
      int i = stack.top();
      // bool found = false;
      for (int j = 0; j < (int)this->A_[i].size(); ++j) {
        if (this->A_[i][j] == 1 && flag.find(j) == flag.end()) {
          stack.push(j);
          flag.insert(j);
          re.emplace_back(astar::Point(this->num2RowCol(j)));
          count++;
          // found = true;
          break; // 处理完一个邻接节点后跳出循环
        }
      }
      if ((int)stack.size() == pre_len) {
        if (count == this->node_num_) {
          break;
        }
        stack.pop(); // 如果没有找到新的节点，则弹出栈顶元素
      }
    }

    std::vector<astar::Point> results = this->completePath(re);
    return results;
  }
};
