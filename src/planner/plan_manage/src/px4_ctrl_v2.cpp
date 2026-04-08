/*****************************************************************************************
 * 自定义控制器V2版本跟踪egoplanner轨迹,速度在0-3m/s，这只是建议速度，当然胆大可以冲4-6m/s，有能力的还可以自己改进下跑跟快
 * 但是最快也就10m/s，当然不是用的这套代码，自己又再改过的，目前没公开
 * 本人目前最快跑到了10m/s，但是需要有先验地图或者上雷达做感知，否则相机的话来不及感知作出避障反应
 * 本代码采用的mavros的速度控制进行跟踪
 * 可以把此cpp放到ego的plan_manage/src下，修改CMakeLists.txt并进行编译,注意:在CMakeList的find_package(catkin REQUIRED COMPONENTS里加入tf和tf2这两个依赖向，否则报错 
 * 编译成功后直接rosrun运行就行，在启动完single_run_in_exp.launch后，rosrun运行本控制代码即可
 * 操作一：遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 * 操作二：遥控器解锁，然后拨到offboard模式，无人机会自己悬停到当前位置1m的高度，然后rviz打点导航
 * 注意！！！：本代码必须要把外部odom融合进飞控才行，或者rtk定位也行；关于PX4融合外部定位的代码可以参考我的gitee
 * gitee在这：https://gitee.com/Canada-a/vins_to_mavros
 * 此外本代码也适合跟踪fastplanner以及用在fuel上，但是强烈建议先把代码的原理看明白再用
 * 本代码卖出去后不退换，要问问题的话B站找我就行，不一定有空回复，纯小白的话建议别用，好好跟着浙大教程来就行，炸炸鸡
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#define PI 3.14159265358979
#define VELOCITY2D_CONTROL 0b011111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
class CXR_CTRL_V2
{
	public:
	//函数
		CXR_CTRL_V2();
		void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg);
		void state_cb(const mavros_msgs::State::ConstPtr& msg);
		void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
		void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);//读取rviz的航点,并触发导航的状态机
		double uav_to_goal(double x, double y);//当前无人机与目标点的距离
		void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);//ego的回调函数，读取ego轨迹前瞻点（即跟踪点）的位置和速度,并把前瞻点发布到rviz可视化出来
		double plan_yaw_rate();//无人机航向角速度计算的函数，就是拿ego的yaw减去无人机的yaw得到角速度，并限制了最大角速度
		geometry_msgs::Point vel_command(double x, double y, double z);//无人机body系下的速度跟踪计算，并把ego的速度1作为前馈加上去，采取了前馈+PD进行跟踪
		void info_state(const ros::TimerEvent &e);//打印信息的状态机
		void main_state(const ros::TimerEvent &e);//整体的运行逻辑部分的函数
		void auto_land_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);


		void wp_target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);//读取waypoint的航点,并触发导航的状态机

		void WGS_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double mag,double &enu_body_x, double &enu_body_y, double &enu_body_z);
		void publish_trigger(const nav_msgs::Odometry &odom_msg);
	//ros
		ros::NodeHandle nh;
		ros::Timer status, zhuantai;
		ros::Subscriber state_sub, rc_sub, twist_sub, target_sub, odom_sub, wp_target_sub, auto_land_sub;
		ros::Publisher pubMarkerPointer, local_pos_pub, pub_extrinsic, traj_start_trigger_pub;
	//msgs
		quadrotor_msgs::PositionCommand ego;
		visualization_msgs::Marker trackpoint;
		tf::StampedTransform ts;//用来发布无人机当前位置的坐标系坐标轴
		tf::TransformBroadcaster tfBroadcasterPointer;	//广播坐标轴
		unsigned short velocity_mask = VELOCITY2D_CONTROL;
		mavros_msgs::PositionTarget current_goal;
		mavros_msgs::RCIn rc;
		std_msgs::Float64MultiArray land_speed;
		int rc_value;
		double mode;
		nav_msgs::Odometry position_msg;
		geometry_msgs::PoseStamped target_pos, wp_target_pos;
		mavros_msgs::State current_state;
		double position_x, position_y, position_z, now_x, now_y, now_z, currvx, currvy, current_yaw, targetpos_x, targetpos_y;
		double ego_pos_x, ego_pos_y, ego_pos_z,feedforward_x, feedforward_y, feedforward_z, ego_yaw,land_speed_x,land_speed_y; //EGO planner information has position
		bool receive, get_now_pos, reach_goal, start_planning_flag, start_landing_flag;//触发轨迹的条件判断
		bool auto_land_flag ;
		geometry_msgs::Point vel;
		double des_vx, des_vy, des_vz, feed_gain;
		//参数调节部分
		const double stop_dist = 0.3;//到终点的时候应当减速的距离，max_vel越大就要设置越大，例如速度3m/s的时候这个stop dist应当要设置到0.7-1m之间
		const double max_vel = 1.0;//和ego参数的max_vel对应，因为ego是基于时间分配的轨迹，所以假如odom卡住的话，这时候轨迹会越来越远，等到odom恢复过来时就和轨迹点距离特别远，导致速度巨大而炸鸡
		const double max_beishu = 1.5;//在限速代码部分我将期望的速度*这个值与期望速度作对比，意味着当期望速度>期望速度*max_beishu，则让期望速度=max_vel;我觉得应该会有更好的方法，靠你们去想了
		//pd这块参数自己看着来，p一般1就行，d的话如果速度在3-4m/s的话，就设置0.2-0.3
		const double px = 1;
		const double dx = 0;
		const double py = 1;
		const double dy = 0.0;//这个很重要，如果ego的max—vel速度在3-4m/s的时候dy应当给到0.4-0.8之间
		const double pz = 1;//z轴只需要p就可以了
		const double yaw_rate_gain = 2;//角速度p,设置3或者4也可以，根据自己需求来
		const double max_yaw_rate = 100;//最大角速度，建议别太低,如果速度在0-1.5m/s之间的话可以设置为80-90，当然100也没问题
};

CXR_CTRL_V2::CXR_CTRL_V2()
{
	receive = false;
	get_now_pos = false;
	start_planning_flag = false;
	auto_land_flag = false;
	start_landing_flag = false;
	status = nh.createTimer(ros::Duration(0.02), &CXR_CTRL_V2::main_state, this);
	zhuantai = nh.createTimer(ros::Duration(1), &CXR_CTRL_V2::info_state, this);
	state_sub = nh.subscribe("/mavros/state", 10, &CXR_CTRL_V2::state_cb, this);//读取飞控状态的话题
	rc_sub=nh.subscribe("/mavros/rc/in",10, &CXR_CTRL_V2::rc_cb, this);//读取遥控器通道的话题，目前不需要
	twist_sub = nh.subscribe("/position_cmd", 10, &CXR_CTRL_V2::twist_cb, this);//订阅egoplanner的规划指令话题的
	target_sub = nh.subscribe("move_base_simple/goal", 10, &CXR_CTRL_V2::target_cb, this);
	odom_sub=nh.subscribe("/mavros/local_position/odom", 10, &CXR_CTRL_V2::odom_cb, this);
	local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pubMarkerPointer = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
	//手动发布外参话题
	pub_extrinsic = nh.advertise<nav_msgs::Odometry>("/extrinsic", 1000);
	//发trigger
	traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

	wp_target_sub = nh.subscribe("/wp_target", 10, &CXR_CTRL_V2::wp_target_cb, this);
	auto_land_sub = nh.subscribe("/auto_land", 10, &CXR_CTRL_V2::auto_land_cb, this);
}

 void CXR_CTRL_V2::auto_land_cb(const std_msgs::Float64MultiArray::ConstPtr&msg)
{
    land_speed = *msg;
	land_speed_x = land_speed.data[0];
	land_speed_y = land_speed.data[1];

	//ROS_INFO("有数据：%f",land_speed_y);
	// if(land_speed_x < 0.00001 && land_speed_y < 0.00001)
	// {
	// 	auto_land_flag = false;
	// }
	// else 
	// {
	// 	auto_land_flag = true;
	// }
}

 void CXR_CTRL_V2::rc_cb(const mavros_msgs::RCIn::ConstPtr&msg)
{
    rc = *msg;
    //rc_value = rc.channels[4];
	mode = ((double)rc.channels[5] - 1000.0) / 1000.0;
		// ROS_INFO("mode=%f",mode);

}

void CXR_CTRL_V2::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

// ksnb
//read vehicle odometry
void CXR_CTRL_V2::odom_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_pos";
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer.sendTransform(ts);
    if(!get_now_pos)
    {
        now_x = msg->pose.pose.position.x;
        now_y = msg->pose.pose.position.y;
		now_z = msg->pose.pose.position.z;
        get_now_pos = true;
		if(now_x > 0.5 || now_x < -0.5 || now_y > 0.5 || now_y < -0.5 )
		{
			ROS_ERROR("里程计原点不是0, 速速重启");
		}
    }
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
    currvx = position_msg.twist.twist.linear.x;
    currvy = position_msg.twist.twist.linear.y;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
	// ROS_INFO("current_yaw=%f",yaw);

}

void CXR_CTRL_V2::target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
  target_pos = *msg;
  targetpos_x = target_pos.pose.position.x;
  targetpos_y = target_pos.pose.position.y;
  receive = true;
}

void CXR_CTRL_V2::wp_target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
  wp_target_pos = *msg;
  targetpos_x = wp_target_pos.pose.position.x;
  targetpos_y = wp_target_pos.pose.position.y;
  
  receive = true;
}

double CXR_CTRL_V2::uav_to_goal(double x, double y)
{
	double dist = sqrt(pow(x - position_x, 2) + pow(y - position_y, 2));
	return dist;
}

void CXR_CTRL_V2::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
    //速度可以作为前馈，根据需要用，这里把ego的速度转化到了机体body系下
    feedforward_x = ego.velocity.x * cos(current_yaw) + ego.velocity.y * sin(current_yaw);
	feedforward_y = -ego.velocity.x*sin(current_yaw) + ego.velocity.y*cos(current_yaw);
	feedforward_z = ego.velocity.z;
    ego_yaw = ego.yaw;
    //跟踪点可视化
    trackpoint.header.frame_id = "world";
    trackpoint.ns = "track_drone";
    trackpoint.id = 0;
    trackpoint.type = visualization_msgs::Marker::SPHERE;
    trackpoint.action = visualization_msgs::Marker::ADD;
    trackpoint.scale.x = 0.3;
    trackpoint.scale.y = 0.3;
    trackpoint.scale.z = 0.3;
    trackpoint.color.a = 1.0;
    trackpoint.color.r = 0.0;
    trackpoint.color.g = 1.0;
    trackpoint.color.b = 0.0;
    trackpoint.pose.position.x = ego_pos_x;
    trackpoint.pose.position.y = ego_pos_y;
    trackpoint.pose.position.z = ego_pos_z;
    pubMarkerPointer.publish(trackpoint);
}

double CXR_CTRL_V2::plan_yaw_rate()
{
	double the; // 极坐标系下的极角
    the = ego_yaw - current_yaw;
    // 限制极角的范围
    if (the > PI)
        the -= 2 * PI;
    else if (the < -PI)
        the += 2 * PI;
    if(the*180/PI > max_yaw_rate)
        the = max_yaw_rate*PI/180;
    else if(the*180/PI < -max_yaw_rate)
        the =-max_yaw_rate*PI/180;
    return the;
}

geometry_msgs::Point CXR_CTRL_V2::vel_command(double x, double y, double z)
{
    vel.x = (x - position_x) * cos(current_yaw) + (y - position_y) * sin(current_yaw);
    vel.y = -(x - position_x)*sin(current_yaw) + (y - position_y)*cos(current_yaw);
    vel.z = z - position_z;
    return vel;
}

void CXR_CTRL_V2::info_state(const ros::TimerEvent &e)
{
	if(!receive)
	{
		ROS_INFO("等待目标点");
	}
	if(receive && !reach_goal)
	{
		ROS_INFO("正在飞往目标点");
	}
	if(reach_goal && receive)
	{
		ROS_INFO("到达目标点");
	}
	// if(mode < 1)
	// {
	// ROS_WARN("请打杆到offboard模式");
	// }
	if(start_landing_flag)
	{
		ROS_INFO("正在降落中......");
	}

	// ROS_INFO("flag=%d",auto_land_flag);
	// ROS_INFO("flag=%d",auto_land_flag);
	// ROS_INFO("flag=%d",auto_land_flag);

}

void CXR_CTRL_V2::main_state(const ros::TimerEvent &e)
{
		/*/////////////////////////////////////
		测试

		/////////////////////////////////////*/
		if((fabs(land_speed_x) > 0.001 && fabs(land_speed_y) > 0.001) || start_landing_flag)//如果收到了降落数据
		{
			if(fabs(land_speed_x) > 0.001 && fabs(land_speed_y) > 0.001)
			{
				current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
				current_goal.header.stamp = ros::Time::now();
				current_goal.type_mask = velocity_mask;
				current_goal.velocity.x = -land_speed_y;
				current_goal.velocity.y = land_speed_x;
				current_goal.velocity.z = -0.1;
				current_goal.yaw_rate = 0;
				start_landing_flag = true;
			}
			else
			{
				current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
				current_goal.header.stamp = ros::Time::now();
				current_goal.type_mask = velocity_mask;
				current_goal.velocity.x = 0;
				current_goal.velocity.y = 0;
				current_goal.velocity.z = -0.1;
				current_goal.yaw_rate = 0;
				
			}

			
		}
		else
		{
			if(((2.0 - position_z) >= 0.2 || (2.0 - position_z <= -0.2) )&& !start_planning_flag)//还没到起飞点
			{
				current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
				current_goal.header.stamp = ros::Time::now();
				current_goal.type_mask = velocity_mask;
				current_goal.velocity.x = (now_x - position_x) * 1;//这里输入了odom第一帧的信息作为起点起飞
				current_goal.velocity.y = (now_y - position_y) * 1;
				current_goal.velocity.z = (2.0 - position_z) * 1;
				current_goal.yaw_rate = 0;
				// ROS_WARN("起飞");

			}
			else//判定为到了起飞点
			{
				if(!start_planning_flag)
				{
					publish_trigger(position_msg);//新加的，用来激活规划
				}
				
				if(!receive)//给规划一点时间，先悬停着
				{
					current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
					current_goal.header.stamp = ros::Time::now();
					current_goal.type_mask = velocity_mask;
					current_goal.velocity.x = 0;//这里输入了odom第一帧的信息作为起点起飞
					current_goal.velocity.y = 0;
					current_goal.velocity.z = 0;
					current_goal.yaw_rate = 0;
				}
				if(receive)
				{
					start_planning_flag = true;
					if(uav_to_goal(targetpos_x, targetpos_y) < (stop_dist + uav_to_goal(ego_pos_x, ego_pos_y)))
					{
						feed_gain = 0;
						reach_goal = true;
					}
					else
					{
						feed_gain = 0.8;
						reach_goal = false;
					}
					//ROS_INFO("targetpos_x=%f,targetpos_y=%f", targetpos_x,targetpos_y);
					des_vx = feed_gain * feedforward_x + px * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).x - dx * currvx;
					des_vy = feed_gain * feedforward_y + py * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).y - dy * currvy;
					des_vz = current_goal.velocity.z =  pz * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).z;
					current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
					current_goal.header.stamp = ros::Time::now();
					current_goal.type_mask = velocity_mask;
					current_goal.yaw_rate = yaw_rate_gain * plan_yaw_rate();
					//============================x==================//
					if(des_vx > max_vel*max_beishu)
					{
						current_goal.velocity.x = max_vel;
					}
					if(des_vx < -max_vel*max_beishu)
					{
						current_goal.velocity.x = -max_vel;
					}
					if(des_vx > -max_vel*max_beishu && des_vx < max_vel*max_beishu)
					{
						current_goal.velocity.x = des_vx;
					}
					//=================y===================================//
					if(des_vy > max_vel*max_beishu)
					{
						current_goal.velocity.y = max_vel;
					}
					if(des_vy < -max_vel*max_beishu)
					{
						current_goal.velocity.y = -max_vel;
					}
					if(des_vy > -max_vel*max_beishu && des_vy < max_vel*max_beishu)
					{
						current_goal.velocity.y = des_vy;
					}
					//===============================z=======================//
					if(des_vz > max_vel)
					{
						current_goal.velocity.z = max_vel;
					}
					if(des_vz < -max_vel)
					{
						current_goal.velocity.z = -max_vel;
					}
					if(des_vz > -max_vel && des_vz < max_vel)
					{
						current_goal.velocity.z = des_vz;
					}
		
				}		
			}
			
		}
		

		////////////////////////////////////


		



		local_pos_pub.publish(current_goal);

	// }

	// //take off 1m
	// if(!receive)
	// {
	// 	current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	// 	current_goal.header.stamp = ros::Time::now();
	// 	current_goal.type_mask = velocity_mask;
	// 	current_goal.velocity.x = (now_x - position_x) * 1;//这里输入了odom第一帧的信息作为起点起飞
	// 	current_goal.velocity.y = (now_y - position_y) * 1;
	// 	// current_goal.velocity.x = 0;
	// 	// current_goal.velocity.y = 0;
	// 	current_goal.velocity.z = (1 - position_z) * 1;
	// 	current_goal.yaw_rate = 0;
	// 	//std::cout<<"ksnb"<<std::endl;

	// 	publish_trigger(position_msg);//新加的，用来激活规划
	// }

	// //if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
	// if(receive)//触发后进行轨迹跟踪
	// {
	// 	if(uav_to_goal(targetpos_x, targetpos_y) < (stop_dist + uav_to_goal(ego_pos_x, ego_pos_y)))
	// 	{
	// 		feed_gain = 0;
	// 		reach_goal = true;
	// 	}
	// 	else
	// 	{
	// 		feed_gain = 0.8;
	// 		reach_goal = false;
	// 	}
	// 	//ROS_INFO("targetpos_x=%f,targetpos_y=%f", targetpos_x,targetpos_y);
	// 	des_vx = feed_gain * feedforward_x + px * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).x - dx * currvx;
	// 	des_vy = feed_gain * feedforward_y + py * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).y - dy * currvy;
	// 	des_vz = current_goal.velocity.z =  pz * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).z;
	// 	current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	// 	current_goal.header.stamp = ros::Time::now();
	// 	current_goal.type_mask = velocity_mask;
	// 	current_goal.yaw_rate = yaw_rate_gain * plan_yaw_rate();
	// 	//============================x==================//
	// 	if(des_vx > max_vel*max_beishu)
	// 	{
	// 		current_goal.velocity.x = max_vel;
	// 	}
	// 	if(des_vx < -max_vel*max_beishu)
	// 	{
	// 		current_goal.velocity.x = -max_vel;
	// 	}
	// 	if(des_vx > -max_vel*max_beishu && des_vx < max_vel*max_beishu)
	// 	{
	// 		current_goal.velocity.x = des_vx;
	// 	}
	// 	//=================y===================================//
	// 	if(des_vy > max_vel*max_beishu)
	// 	{
	// 		current_goal.velocity.y = max_vel;
	// 	}
	// 	if(des_vy < -max_vel*max_beishu)
	// 	{
	// 		current_goal.velocity.y = -max_vel;
	// 	}
	// 	if(des_vy > -max_vel*max_beishu && des_vy < max_vel*max_beishu)
	// 	{
	// 		current_goal.velocity.y = des_vy;
	// 	}
	// 	//===============================z=======================//
	// 	if(des_vz > max_vel)
	// 	{
	// 		current_goal.velocity.z = max_vel;
	// 	}
	// 	if(des_vz < -max_vel)
	// 	{
	// 		current_goal.velocity.z = -max_vel;
	// 	}
	// 	if(des_vz > -max_vel && des_vz < max_vel)
	// 	{
	// 		current_goal.velocity.z = des_vz;
	// 	}
	// }

	// /////////////////////////////////////////////
	// nav_msgs::Odometry odometry;
    // //odometry.header = header;
	// odometry.header.stamp = ros::Time::now();
    // odometry.header.frame_id = "world";
    // odometry.pose.pose.position.x = 0.06971589185939399;
    // odometry.pose.pose.position.y = 0.017252895372223007;
    // odometry.pose.pose.position.z = -0.03453168694252692;
    // odometry.pose.pose.orientation.x = 0.5153391069608364;
    // odometry.pose.pose.orientation.y = -0.5016761082562277;
    // odometry.pose.pose.orientation.z = 0.5011301468896686;
    // odometry.pose.pose.orientation.w = -0.4812642341999172;
    // pub_extrinsic.publish(odometry);
	// /////////////////////////////////////////////////////

	// local_pos_pub.publish(current_goal);



}

void CXR_CTRL_V2::WGS_TO_ENU(double lat, double lon, double h, 
							 double lat0, double lon0, double h0, double mag,
							 double &enu_body_x, double &enu_body_y, double &enu_body_z)
{
	double a, b, f, e_sq, pi;
    pi = 3.14159265359;
	a = 6378137;
	b = 6356752.3142;
	f = (a - b) / a;
	e_sq = f * (2 - f);
	// 站点（非原点）
	double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
	lamb = lat/180.0*pi;  
	phi = lon/180.0*pi;
	s = sin(lamb);
	N = a / sqrt(1 - e_sq * s * s);
 
	sin_lambda = sin(lamb);
	cos_lambda = cos(lamb);
	sin_phi = sin(phi);
	cos_phi = cos(phi);
 
	x = (h + N) * cos_lambda * cos_phi;
	y = (h + N) * cos_lambda * sin_phi;
	z = (h + (1 - e_sq) * N) * sin_lambda;
	// 原点坐标转换
	double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
	lamb0 = lat0/180.0*pi;
	phi0 = lon0/180.0*pi;
	s0 = sin(lamb0);
	N0 = a / sqrt(1 - e_sq * s0 * s0);
 
	sin_lambda0 = sin(lamb0);
	cos_lambda0 = cos(lamb0);
	sin_phi0 = sin(phi0);
	cos_phi0 = cos(phi0);
 
	x0 = (h0 + N0) * cos_lambda0 * cos_phi0;
	y0 = (h0 + N0) * cos_lambda0 * sin_phi0;
	z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0;
	// ECEF 转 ENU
	double xd, yd, zd, t;
	xd = x - x0;
	yd = y - y0;
	zd = z - z0;
	t = -cos_phi0 * xd - sin_phi0 * yd;
 
	double enu_world_x,enu_world_y,enu_world_z,yaw_angle;
	enu_world_x = -sin_phi0 * xd + cos_phi0 * yd;
	enu_world_y = t * sin_lambda0 + cos_lambda0 * zd;
	enu_world_z = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd;

///////////////////////////////////////////////////////////////////////////////////////////
	//输入mag是角度
	yaw_angle = -((mag * pi)/180 - pi/2); //以弧度为单位，45度示例,以逆时针为正

	//应用旋转矩阵进行坐标转换
	enu_body_x =  enu_world_x * cos(yaw_angle) + enu_world_y * sin(yaw_angle);
	enu_body_y = -enu_world_x * sin(yaw_angle) + enu_world_y * cos(yaw_angle);
	enu_body_z =  enu_world_z;
}

void CXR_CTRL_V2::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "px4_ctrl_v2");
	setlocale(LC_ALL,""); 
	CXR_CTRL_V2 cxr_ctrl_v2;
	ros::spin();
	return 0;
}

