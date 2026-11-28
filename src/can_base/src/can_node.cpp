#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "can_base/msg/can_frame.hpp"

class CANNode : public rclcpp::Node
{
public:
	CANNode() :
		Node("can_node")
	{
		auto callback = 
			[this](can_base::msg::CanFrame::UniquePtr msg) -> void
			{
				RCLCPP_INFO(	this->get_logger(),
                        "Sent CAN frame");
			};

		sub =
			this->create_subscription<can_base::msg::CanFrame>(
					"can_msgs", 10, callback);
	}

	~CANNode()
	{
		RCLCPP_INFO(	this->get_logger(),
				"Reached destructor\n");
        close(sock);
	}

private:
	rclcpp::Subscription<can_base::msg::CanFrame>::SharedPtr sub;
	int sock;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	try {
		rclcpp::spin(std::make_shared<CANNode>());
	} catch (std::runtime_error &e) {
		std::cerr << e.what();
		return 1;
	}

	rclcpp::shutdown();

	return 0;
}
