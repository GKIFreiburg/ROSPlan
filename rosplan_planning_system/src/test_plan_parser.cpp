
#include <ros/ros.h>
#include <rosplan_dispatch_msgs/CompletePlan.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_planning_system/POPFEsterelPlanParser.h>
#include <rosplan_planning_system/PlanningEnvironment.h>

#include <cstdio>

using namespace KCL_rosplan;
std::string data_path = "/home/andreas/plan_test/";


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_plan_parser");

	ros::NodeHandle n;
	PlanningEnvironment environment;
	environment.update(n);
	environment.parseDomain(data_path+"domain.pddl");
	POPFEsterelPlanParser parser(n);
	parser.preparePlan(data_path, environment, 0);

	//ros::spin();

	return 0;
}
