#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <vector>
#include <memory>

using std::placeholders::_1;

class SimpleParameter: public rclcpp::Node {
  public:
    SimpleParameter() : Node("simple_parameter") {
      declare_parameter<int>("simple_int_param", 22);
      declare_parameter<std::string>("simple_string_param", "Maddog");

      param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

  private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;

      for (const auto& param : params) {
        if (param.get_name() == "simple_int_pram" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed: " << param.as_int());
          result.successful = true;
        } else if (param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed: " << param.as_string());
          result.successful = true;
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Param " << param.get_name() << " does not exist!!!");
          result.successful = false;
        }
      }

      return result;
    }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimpleParameter>();
  rclcpp::spin(node);
  rclcpp::shutdown();


  return 0;
}