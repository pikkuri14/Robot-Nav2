#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("navigate_bt") + "/bt_xml";

class myBtNode : public rclcpp::Node
{
public:
  explicit myBtNode(const std::string &node_name);
  void setup();
  void create_behavior_tree();
  void update_behavior_tree();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  BT::Tree tree_;
};

class ApproachObject : public BT::SyncActionNode
{
public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {}

    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject" << this-name() <<std::endl;
    }
}

class CheckBattery: : public BT::SyncActionNode
{
public:
    CheckBattery:(const std::string& name) :
        BT::SyncActionNode(name, {})
    {}

    BT::NodeStatus tick() override
    {
        std::cout << "CheckBattery: OK" << std::endl;
    }
}





myBtNode::myBtNode(const std::string &nodeName) : Node(nodeName)
{
  this->declare_parameter("location_file","none");

  RCLCPP_INFO(get_logger(), "Init done");
}

void myBtNode::setup()
{
  RCLCPP_INFO(get_logger(), "Setting up");
  create_behavior_tree();
  RCLCPP_INFO(get_logger(), "BT created");

  const auto timer_period = 500ms;
  timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&myBtNode::update_behavior_tree, this));

  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

void myBtNode::create_behavior_tree()
{
  BT::BehaviorTreeFactory factory;

  // register bt node

  factory.registerBuilder<GoToPose>("GoToPose", builder);

  RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());

  tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
  RCLCPP_INFO(get_logger(), "3");
}

void myBtNode::update_behavior_tree()
{
  BT::NodeStatus tree_status = tree_.tickRoot();

  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return;
  }
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Finished Navigation");
  }
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Failed");
    timer_->cancel();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<myBtNode>("autonomy_node");
  node->setup();

  return 0;
}
