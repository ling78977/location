#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "GraphicsMagick/Magick++/Image.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "yaml-cpp/yaml.h"
// #include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <string>
std::string frame_id;
std::string topic_name;
std::string yaml_filename;
static char *dirname(char *path) {
  static char dot[] = ".";
  char *last_slash;

  if (path == NULL) {
    return path;
  }

  /* Replace all "\" with "/" */
  char *c = path;
  while (*c != '\0') {
    if (*c == '\\') {
      *c = '/';
    }
    ++c;
  }

  /* Find last '/'.  */
  last_slash = path != NULL ? strrchr(path, '/') : NULL;

  if (last_slash != NULL && last_slash == path) {
    /* The last slash is the first character in the string.  We have to
       return "/".  */
    ++last_slash;
  } else if (last_slash != NULL && last_slash[1] == '\0') {
    /* The '/' is the last character, we have to look further.  */
    last_slash = reinterpret_cast<char *>(memchr(path, last_slash - path, '/'));
  }

  if (last_slash != NULL) {
    /* Terminate the path.  */
    last_slash[0] = '\0';
  } else {
    /* This assignment is ill-designed but the XPG specs require to
       return a string containing "." in any case no directory part is
       found and so a static and constant string is required.  */
    path = reinterpret_cast<char *>(dot);
  }

  return path;
}

enum class MapMode {
  /**
   * Together with associated threshold values (occupied and free):
   *   lightness >= occupied threshold - Occupied (100)
   *             ... (anything in between) - Unknown (-1)
   *    lightness <= free threshold - Free (0)
   */
  Trinary,
  /**
   * Together with associated threshold values (occupied and free):
   *   alpha < 1.0 - Unknown (-1)
   *   lightness >= occ_th - Occupied (100)
   *             ... (linearly interpolate to)
   *   lightness <= free_th - Free (0)
   */
  Scale,
  /**
   * Lightness = 0 - Free (0)
   *          ... (linearly interpolate to)
   * Lightness = 100 - Occupied (100)
   * Lightness >= 101 - Unknown
   */
  Raw,
};
struct LoadParameters {
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

typedef enum {
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;

// === Map input part ===

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in
/// slightly nicer error messages, including the name of the failed key
/// @throw YAML::Exception
template <typename T>
T yaml_get_value(const YAML::Node &node, const std::string &key) {
  try {
    return node[key].as<T>();
  } catch (YAML::Exception &e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}
MapMode map_mode_from_string(std::string map_mode_name) {
  for (auto &c : map_mode_name) {
    c = tolower(c);
  }

  if (map_mode_name == "scale") {
    return MapMode::Scale;
  } else if (map_mode_name == "raw") {
    return MapMode::Raw;
  } else if (map_mode_name == "trinary") {
    return MapMode::Trinary;
  } else {
    throw std::invalid_argument("map_mode_name");
  }
}
const char *map_mode_to_string(MapMode map_mode) {
  switch (map_mode) {
  case MapMode::Trinary:
    return "trinary";
  case MapMode::Scale:
    return "scale";
  case MapMode::Raw:
    return "raw";
  default:
    throw std::invalid_argument("map_mode");
  }
}
// using nav2_util::geometry_utils::orientationAroundZAxis;

LoadParameters loadMapYaml(const std::string &yaml_filename) {
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name =
        std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
        doc["origin"].Mark(),
        "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh =
      yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode =
        map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  std::cout << "[DEBUG] [map_io]: resolution: " << load_parameters.resolution
            << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[0]: " << load_parameters.origin[0]
            << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[1]: " << load_parameters.origin[1]
            << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[2]: " << load_parameters.origin[2]
            << std::endl;
  std::cout << "[DEBUG] [map_io]: free_thresh: " << load_parameters.free_thresh
            << std::endl;
  std::cout << "[DEBUG] [map_io]: occupied_thresh: "
            << load_parameters.occupied_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: mode: "
            << map_mode_to_string(load_parameters.mode) << std::endl;
  std::cout << "[DEBUG] [map_io]: negate: " << load_parameters.negate
            << std::endl; // NOLINT

  return load_parameters;
}

void loadMapFromFile(const LoadParameters &load_parameters,
                     nav_msgs::msg::OccupancyGrid &map) {
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;

  std::cout << "[INFO] [map_io]: Loading image_file: "
            << load_parameters.image_file_name << std::endl;
  Magick::Image img(load_parameters.image_file_name);

  // Copy the image data into the map structure
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();
  msg.info.resolution = load_parameters.resolution;
  msg.info.origin.position.x = load_parameters.origin[0];
  msg.info.origin.position.y = load_parameters.origin[1];
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(
          load_parameters.origin[2]);

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  for (size_t y = 0; y < msg.info.height; y++) {
    for (size_t x = 0; x < msg.info.width; x++) {
      auto pixel = img.pixelColor(x, y);
      std::vector<Magick::Quantum> channels = {
          pixel.redQuantum(), pixel.greenQuantum(), pixel.blueQuantum()};
      if (load_parameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color
        // channels in Trinary mode. CAREFUL. alpha is inverted from what
        // you might expect. High = transparent, low = opaque
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      double shade =
          Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());
      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before
      /// thresholding)?
      double occ = (load_parameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (load_parameters.mode) {
      case MapMode::Trinary:
        if (load_parameters.occupied_thresh < occ) {
          map_cell = nav2_util::OCC_GRID_OCCUPIED;
        } else if (occ < load_parameters.free_thresh) {
          map_cell = nav2_util::OCC_GRID_FREE;
        } else {
          map_cell = nav2_util::OCC_GRID_UNKNOWN;
        }
        break;
      case MapMode::Scale:
        if (pixel.alphaQuantum() != OpaqueOpacity) {
          map_cell = nav2_util::OCC_GRID_UNKNOWN;
        } else if (load_parameters.occupied_thresh < occ) {
          map_cell = nav2_util::OCC_GRID_OCCUPIED;
        } else if (occ < load_parameters.free_thresh) {
          map_cell = nav2_util::OCC_GRID_FREE;
        } else {
          map_cell = std::rint(
              (occ - load_parameters.free_thresh) /
              (load_parameters.occupied_thresh - load_parameters.free_thresh) *
              100.0);
        }
        break;
      case MapMode::Raw: {
        double occ_percent = std::round(shade * 255);
        if (nav2_util::OCC_GRID_FREE <= occ_percent &&
            occ_percent <= nav2_util::OCC_GRID_OCCUPIED) {
          map_cell = static_cast<int8_t>(occ_percent);
        } else {
          map_cell = nav2_util::OCC_GRID_UNKNOWN;
        }
        break;
      }
      default:
        throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
    }
  }

  // Since loadMapFromFile() does not belong to any node, publishing in a
  // system time.
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  msg.info.map_load_time = clock.now();
  msg.header.frame_id = frame_id;
  msg.header.stamp = clock.now();

  std::cout << "[DEBUG] [map_io]: Read map " << load_parameters.image_file_name
            << ": " << msg.info.width << " X " << msg.info.height << " map @ "
            << msg.info.resolution << " m/cell" << std::endl;

  map = msg;
}

LOAD_MAP_STATUS loadMapFromYaml(const std::string &yaml_file,
                                nav_msgs::msg::OccupancyGrid &map) {
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io]: YAML file name is empty, can't load!"
              << std::endl;
    return MAP_DOES_NOT_EXIST;
  }
  std::cout << "[INFO] [map_io]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed processing YAML file " << yaml_file
              << " at position (" << e.mark.line << ":" << e.mark.column
              << ") for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  } catch (std::exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed to parse map YAML loaded from file "
              << yaml_file << " for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  }
  try {
    loadMapFromFile(load_parameters, map);
  } catch (std::exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed to load image file "
              << load_parameters.image_file_name << " for reason: " << e.what()
              << std::endl;
    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}
class MapPublisher : public rclcpp::Node {
public:
  MapPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("map_publisher", options) {
        RCLCPP_INFO(this->get_logger(), "开始地图发布！");
    this->declare_parameter<std::string>("frame_id", "camera_init");
    this->declare_parameter<std::string>("topic_name", "map");
    this->declare_parameter<std::string>("yaml_filename", "");
    this->get_parameter_or<std::string>("frame_id", frame_id, "camera_init");
    this->get_parameter_or<std::string>("topic_name", topic_name, "map");
    this->get_parameter_or<std::string>("yaml_filename", yaml_filename, "");
    auto status = loadMapFromYaml(yaml_filename, this->map_);
    if (status != LOAD_MAP_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "加载地图文件失败！");
      return;
    }
    map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name, 10);
    //------------------------------------------------------------------------------------------------------
    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    timer_ =
        rclcpp::create_timer(this, this->get_clock(), period_ms,
                             std::bind(&MapPublisher::time_callback, this));
  }

private:
  void time_callback() { map_publisher_->publish(map_); }
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid map_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  if (rclcpp::ok())
    rclcpp::shutdown();


  return 0;
}