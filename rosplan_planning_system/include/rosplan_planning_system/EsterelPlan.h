#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <ostream>
#include <vector>

#ifndef KCL_esterel_plan
#define KCL_esterel_plan

namespace KCL_rosplan
{

	enum SignalType { ACTION, CONDITION, TIME };

	struct StrlNode;
	
	/* Plan edge definition */
	struct StrlEdge
	{
		SignalType signal_type;
		std::string edge_name;
		std::vector<StrlNode*> sources;
		std::vector<StrlNode*> sinks;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> external_conditions;
		bool active;
	};
	
	/* Plan node definition */
	struct StrlNode
	{
		std::string node_name;
		int node_id;
		std::vector<StrlEdge*> input;
		std::vector<StrlEdge*> output;

		std::map<StrlEdge*, bool> await_input;
		bool dispatched;
		bool completed;

		rosplan_dispatch_msgs::ActionDispatch dispatch_msg;
		StrlNode()
		{
			node_id = -1;
			dispatched = false;
			completed = false;
		}

		void activateInput(StrlEdge* signal);
		bool allInputsActive() const;
		void reset();
	};
}

std::ostream& operator<<(std::ostream& os, const KCL_rosplan::StrlEdge& edge);
std::ostream& operator<<(std::ostream& os, const KCL_rosplan::StrlNode& node);

#endif
