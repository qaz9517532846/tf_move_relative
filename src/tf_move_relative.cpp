#include <tf_move_relative/tf_move_relative.h>

tf_MoveRelative::tf_MoveRelative(std::string name) : action_server_(nh_, 
                                                                    name,
                                                                    boost::bind(&tf_MoveRelative::execute, this, _1),
                                                                    false)
{
    ros::NodeHandle private_nh;
    
    goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
    vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    dynamic_reconfigure::Server<tf_move_relative::tf_MoveRelativeConfig>::CallbackType f;

    f = boost::bind(&tf_MoveRelative::reconfig_callback, this, _1, _2);
    reconfigure_server_.setCallback(f);

    tf_listener_ = new tf2_ros::TransformListener(tf_Buffer_);

    action_server_.start();
}

tf_MoveRelative::~tf_MoveRelative()
{

}

void tf_MoveRelative::reconfig_callback(tf_move_relative::tf_MoveRelativeConfig &config, uint32_t level)
{
    rate_ = config.rate;
    timeout_ = config.timeout;
    base_frame_ = config.base_frame;
    fixed_frame_ = config.fixed_frame;
    x_.max_vel = config.max_vel_x;
    x_.min_vel = config.min_vel_x;
    y_.max_vel = config.max_vel_y;
    y_.min_vel = config.min_vel_y;
    theta_.max_vel = config.max_vel_theta;
    theta_.min_vel = config.min_vel_theta;
    x_.acceleration = config.acceleration_x;
    y_.acceleration = config.acceleration_y;
    theta_.acceleration = config.acceleration_theta;
    linear_tolerance_sq_ = config.linear_tolerance * config.linear_tolerance;
    angular_tolerance_ = config.angular_tolerance;
}

void tf_MoveRelative::execute(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    ROS_INFO("Recieved goal");
    move_base_msgs::MoveBaseResult result;
    move_base_msgs::MoveBaseFeedback feedback;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::TransformStamped move_relative_fixed_transform;

    tf2::Stamped<tf2::Transform> move_relative_fixed_tf2;
    tf2::Stamped<tf2::Transform> goal_tf2;

    ros::Rate r(rate_);
    ros::Duration timeout = ros::Duration(timeout_);

    std::string move_relative_frame_ = goal->target_pose.header.frame_id;
    ROS_INFO("Move_relative_frame = %s", move_relative_frame_.c_str());

    goal_pub_.publish(goal->target_pose);

    try
    {
      tf_Buffer_.transform(goal->target_pose, goal_pose, move_relative_frame_); //goal->target_pose relative to move_relative_frame_ convert goal_pose
      
      // lookupTransform (parent, child, ...)
      move_relative_fixed_transform = tf_Buffer_.lookupTransform(fixed_frame_, move_relative_frame_, ros::Time::now(), timeout);

      tf2::convert(move_relative_fixed_transform, move_relative_fixed_tf2); // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
      tf2::convert(goal_pose, goal_tf2);

      if(x_.acceleration == 0 || y_.acceleration == 0 || theta_.acceleration == 0)
      {
        ROS_ERROR_STREAM("Abort because of set acceleration is zero.");
        return;
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Abort, goal invalid. " << ex.what());
      action_server_.setAborted(result, ex.what());
      return;
    }

    x_.current_vel = 0;
    y_.current_vel = 0;
    theta_.current_vel = 0;

    while(ros::ok())
    {
      if (action_server_.isPreemptRequested())
      {
        stop_vel();
        ROS_WARN_STREAM("Action canceled.");
        action_server_.setPreempted(result, "Action canceled");
        return;
      }

      geometry_msgs::TransformStamped fixed_base_transform;

      try
      {
        last_time_ = ros::Time::now();
        // lookupTransform (parent, child, ...)
        fixed_base_transform = tf_Buffer_.lookupTransform(base_frame_, fixed_frame_, last_time_, timeout);
      }
      catch (tf2::TransformException &ex)
      {
        stop_vel();
        ROS_ERROR_STREAM("Abort. " << ex.what());
        action_server_.setAborted(result, ex.what());
        return;
      }

      tf2::Stamped<tf2::Transform> fixed_base_tf2;

      // convert geometry_msgs::PoseStamped &msg to tf2::Stamped< tf2::Transform >
      tf2::convert(fixed_base_transform, fixed_base_tf2);

      tf2::Transform diff_tf2;

      diff_tf2 = fixed_base_tf2 * move_relative_fixed_tf2 * goal_tf2;

      double diff_x = diff_tf2.getOrigin().x();
      double diff_y = diff_tf2.getOrigin().y();
      double diff_yaw = tf2::getYaw(diff_tf2.getRotation());
      //ROS_INFO("diff_x = %f, diff_y = %f, diff_th = %f", diff_x, diff_y, diff_yaw);
      if (diff_x * diff_x + diff_y * diff_y <= linear_tolerance_sq_ && fabs(diff_yaw) <= angular_tolerance_)
      {
        ROS_INFO_STREAM("diff_tf2.getOrigin().x(): " << diff_tf2.getOrigin().x() << " diff_tf2.getOrigin().y(): " << diff_tf2.getOrigin().y() );
        stop_vel();
        ROS_INFO_STREAM("Success");
        action_server_.setSucceeded(result, "Success");
        return;
      }

      feedback.base_position.header =  goal->target_pose.header;
      feedback.base_position.header.frame_id =  base_frame_;
      feedback.base_position.pose.position.x = diff_x;
      feedback.base_position.pose.position.x = diff_y;
      tf2::Quaternion quat;
      quat.setRPY(0, 0, diff_yaw);
      feedback.base_position.pose.orientation.w = quat.w();
      feedback.base_position.pose.orientation.x = quat.x();
      feedback.base_position.pose.orientation.y = quat.y();
      feedback.base_position.pose.orientation.z = quat.z();

      // tf2::toMsg(fixed_reference_tf2, feedback.base_position); // this works
      // tf2::convert(fixed_reference_tf2, feedback.base_position); // this does not work, why?
      action_server_.publishFeedback(feedback);

      geometry_msgs::Twist pub_vel;
      pub_vel.linear.x = cal_vel(diff_x, x_);
      pub_vel.linear.y = cal_vel(diff_y, y_);
      pub_vel.angular.z = cal_vel(diff_yaw, theta_);
      vel_pub_.publish(pub_vel);

      r.sleep();
    }
}

void tf_MoveRelative::stop_vel()
{
    geometry_msgs::Twist pub_vel;
    vel_pub_.publish(pub_vel);
}

double tf_MoveRelative::cal_vel(double difference, velocity_setting &set)
{
  double velocity;
  double dt = (ros::Time::now() - last_time_).toSec();

  double sign = difference < 0 ? -1 : 1;
  
  velocity = sign * sqrt(fabs(difference) * 2 * set.acceleration);

  if(velocity > set.current_vel)
	{
		set.current_vel += fmin(velocity - set.current_vel, set.acceleration * dt);
	}
	else
	{
		set.current_vel += fmax(velocity - set.current_vel, -set.acceleration * dt);
	}

  if(set.current_vel > set.max_vel)
  {
    set.current_vel = set.max_vel;
  }
  else if(set.current_vel < set.min_vel)
  {
    set.current_vel = set.min_vel;
  }

  return set.current_vel;
}