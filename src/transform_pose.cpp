#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <find_object_2d/msg/objects_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <QtCore/QString>

class TfPose : public rclcpp::Node
{
public:
	TfPose() :
		Node("tf_pose_node"),
		objFramePrefix_("object")
	{
		tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		//	this->get_node_base_interface(),
		//	this->get_node_timers_interface());
		//tfBuffer_->setCreateTimerInterface(timer_interface);
		tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("tf_pose", 10);

		targetFrameId_ = this->declare_parameter("target_frame_id", targetFrameId_);
		objFramePrefix_ = this->declare_parameter("object_prefix", objFramePrefix_);

		subs_ = create_subscription<find_object_2d::msg::ObjectsStamped>("objectsStamped", rclcpp::QoS(5).reliability((rmw_qos_reliability_policy_t)1), std::bind(&TfPose::objectsDetectedCallback, this, std::placeholders::_1));
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::msg::ObjectsStamped::ConstSharedPtr msg)
	{
		if(msg->objects.data.size())
		{
			std::string targetFrameId = targetFrameId_;
			if(targetFrameId.empty())
			{
				targetFrameId = msg->header.frame_id;
			}
			char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				// int id = (int)msg->objects.data[i];

				// QString multiSuffix;
				// if(id == previousId)
				// {
				// 	multiSuffix = QString("_") + multiSubId++;
				// }
				// else
				// {
				// 	multiSubId = 'b';
				// }
				// previousId = id;

				// "object_1", "object_1_b", "object_1_c", "object_2"
				// std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
                std::string objectFrameId = "object_"+std::to_string((int)msg->objects.data[i]);

				geometry_msgs::msg::TransformStamped pose;
				try
				{
					// Get transformation from "object_#" frame to target frame
					// The timestamp matches the one sent over TF
					pose = tfBuffer_->lookupTransform(targetFrameId, objectFrameId, tf2_ros::fromMsg(msg->header.stamp));
				}
				catch(tf2::TransformException & ex)
				{
					RCLCPP_WARN(this->get_logger(), "%s",ex.what());
					continue;
				}

                geometry_msgs::msg::PoseStamped object_pose;
                object_pose.header.frame_id = objectFrameId.c_str();
                object_pose.pose.position.x = pose.transform.translation.x;
                object_pose.pose.position.y = pose.transform.translation.y;
                object_pose.pose.position.z = pose.transform.translation.z;
                object_pose.pose.orientation.x = pose.transform.rotation.x;
                object_pose.pose.orientation.y = pose.transform.rotation.y;
                object_pose.pose.orientation.z = pose.transform.rotation.z;
                object_pose.pose.orientation.w = pose.transform.rotation.w;
                pub->publish(object_pose);

				// Here "pose" is the position of the object "id" in target frame.
				// RCLCPP_INFO(this->get_logger(), "%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
				// 		objectFrameId.c_str(), targetFrameId.c_str(),
				// 		pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z,
				// 		pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w);
			}
		}
	}

private:
	std::string targetFrameId_;
	std::string objFramePrefix_;
	rclcpp::Subscription<find_object_2d::msg::ObjectsStamped>::SharedPtr subs_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TfPose>());
	rclcpp::shutdown();
}