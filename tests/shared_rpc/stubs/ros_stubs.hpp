#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
namespace rclcpp {
struct Time {};
struct Duration { double seconds() const { return 0.008; } };
inline int get_logger(const char*) { return 0; }
template<class T> void sleep_for(T) {}
}
namespace rclcpp_lifecycle { struct State {}; }
#define RCLCPP_SHARED_PTR_DEFINITIONS(...)
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_ERROR_ONCE(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_FATAL(...) ((void)0)
#define PLUGINLIB_EXPORT_CLASS(...)
#define FAIRINO_HARDWARE_PUBLIC
namespace hardware_interface {
enum class CallbackReturn { SUCCESS, ERROR };
enum class return_type { OK, ERROR };
inline const std::string HW_IF_POSITION = "position";
inline const std::string HW_IF_VELOCITY = "velocity";
struct InterfaceInfo { std::string name; };
struct ComponentInfo {
    std::string name;
    std::vector<InterfaceInfo> command_interfaces, state_interfaces;
};
struct HardwareInfo {
    std::vector<ComponentInfo> joints;
    std::map<std::string, std::string> hardware_parameters;
};
struct StateInterface {
    double * value;
    StateInterface(const std::string&, const std::string&, double * p) : value(p) {}
};
struct CommandInterface {
    double * value;
    CommandInterface(const std::string&, const std::string&, double * p) : value(p) {}
};
class SystemInterface {
public:
    virtual ~SystemInterface() = default;
    virtual CallbackReturn on_init(const HardwareInfo & info) { info_ = info; return CallbackReturn::SUCCESS; }
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State&) = 0;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) = 0;
    virtual std::vector<StateInterface> export_state_interfaces() = 0;
    virtual std::vector<CommandInterface> export_command_interfaces() = 0;
    virtual return_type read(const rclcpp::Time&, const rclcpp::Duration&) = 0;
    virtual return_type write(const rclcpp::Time&, const rclcpp::Duration&) = 0;
protected:
    HardwareInfo info_;
};
}
