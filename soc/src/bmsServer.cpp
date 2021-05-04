#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <soc/BatteryAction.h>
#include <soc/Current.h>
#include <iostream>
#include <fstream>

constexpr int MAX_BATTERY_CAPACITY = 5*60*60;

class BmsServer
{
enum BatteryMode
{
  charging = 0,
  discharging
};

enum BatteryState
{
  OK = 0,
  NOK
};

public:
  BmsServer(std::string name) : 
    as_(nh_, name, false),
    action_name_(name),
    AsSOC(MAX_BATTERY_CAPACITY * 0.6),
    dataCount(0),
    goalSample(0),
    chargingLimit(5),
    dischargingLimit(10)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&BmsServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&BmsServer::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/actualCurrent", 1, &BmsServer::analysisCB, this);
    as_.start();

    //discard csv files
    feedbackFile.open ("/home/kamil/catkin_ws/src/soc/src/feedback.csv", std::ofstream::trunc);
    feedbackFile << "batteryState,batteryMode,procentSOC,mAhSOC\n";
    feedbackFile.close();
    resultFile.open ("/home/kamil/catkin_ws/src/soc/src/result.csv", std::ofstream::trunc);
    resultFile << "estTimetoFull,estTimeToEmpty\n";
    resultFile.close();
    currentFile.open ("/home/kamil/catkin_ws/src/soc/src/current.csv", std::ofstream::trunc);
    currentFile << "AsSOC,current,diffTime\n";
    currentFile.close();
  }

  ~BmsServer(void)
  {
  }

  void goalCB()
  {
    // reset helper variables
    dataCount = 0;
    // accept the new goal
    goalSample = as_.acceptNewGoal()->samples;
    ROS_INFO("New goal sample is:%d", goalSample);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const soc::Current::ConstPtr& msg)
  {
    if (AsSOC + msg->current * msg->time < MAX_BATTERY_CAPACITY)
    {
      AsSOC += msg->current * msg->time;
    }
    saveCurrentAsCsv(msg->current, msg->time);

    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;

    if (dataCount < MAX_LIMIT_OF_SAMPLES)
    {
      measuredCurrent.at(dataCount) = msg->current;
      dataCount++;
    }

    feedback_.procentSOC = (AsSOC / MAX_BATTERY_CAPACITY) * 100;
    feedback_.mAhSOC = ((AsSOC / 60) / 60) * 1000;
    feedback_.batteryMode = msg->current < 0 ? BatteryMode::discharging : BatteryMode::charging;
    if (feedback_.batteryMode == BatteryMode::discharging)
    {
      feedback_.batteryState = (-msg->current) < dischargingLimit ? BatteryState::OK : BatteryState::NOK;
    }
    else
    {
      feedback_.batteryState = msg->current < chargingLimit ? BatteryState::OK : BatteryState::NOK;
    }
    saveFeedbackAsCsv();
    as_.publishFeedback(feedback_);

    if(dataCount > goalSample) 
    {
      float sumCurrent = 0;
      for (int i = 0; i < dataCount; i++)
      {
        sumCurrent += measuredCurrent.at(i);
      }
      result_.estTimetoFull = (MAX_BATTERY_CAPACITY - AsSOC) / (sumCurrent / dataCount);
      result_.estTimeToEmpty = AsSOC / (sumCurrent / dataCount);
      saveResultAsCsv();
      as_.setSucceeded(result_);
    }
  }

  void saveFeedbackAsCsv()
  {
    feedbackFile.open ("/home/kamil/catkin_ws/src/soc/src/feedback.csv", std::ofstream::app);
    feedbackFile << feedback_.batteryState << ',' << feedback_.batteryMode << ',' << feedback_.procentSOC << ',' << feedback_.mAhSOC << std::endl;
    feedbackFile.close();
  }

  void saveResultAsCsv()
  {
    resultFile.open ("/home/kamil/catkin_ws/src/soc/src/result.csv", std::ofstream::app);
    resultFile << result_.estTimetoFull << ',' << result_.estTimeToEmpty << std::endl;
    resultFile.close();
  }

  void saveCurrentAsCsv(float current, float diffTime)
  {
    currentFile.open ("/home/kamil/catkin_ws/src/soc/src/current.csv", std::ofstream::app);
    currentFile << AsSOC << ',' << current << ',' << diffTime << std::endl;
    currentFile.close();
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<soc::BatteryAction> as_;
  std::string action_name_;
  soc::BatteryFeedback feedback_;
  soc::BatteryResult result_;
  ros::Subscriber sub_;

private:
  float AsSOC;
  int dataCount;
  int goalSample;
  int chargingLimit;
  int dischargingLimit;
  const int MAX_LIMIT_OF_SAMPLES = 1000;
  std::array<float, 1000> measuredCurrent;
  std::ofstream feedbackFile;
  std::ofstream resultFile;
  std::ofstream currentFile;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery");

  BmsServer battery(ros::this_node::getName());
  ros::spin();

  return 0;
}
