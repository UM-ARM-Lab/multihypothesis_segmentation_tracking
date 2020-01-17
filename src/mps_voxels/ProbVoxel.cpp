//
// Created by kunhuang on 10/23/19.
//

#include "mps_voxels/ProbVoxel.h"

namespace octomap {

OcTreeLabel::OcTreeLabel(double in_resolution)
	: OccupancyOcTreeBase<OcTreeNodeLabel>(in_resolution) {
	ocTreeLabelMemberInit.ensureLinking();
}

//unsigned int OcTreeLabel::getLastUpdateTime() {
//	// this value is updated whenever inner nodes are
//	// updated using updateOccupancyChildren()
//	return root->getLabel();
//}
//
//void OcTreeLabel::degradeOutdatedNodes(unsigned int time_thres) {
//	unsigned int query_time = (unsigned int) time(NULL);
//
//	for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
//	    it!= end; ++it) {
//		if ( this->isNodeOccupied(*it)
//		     && ((query_time - it->getLabel()) > time_thres) ) {
//			integrateMissNoTime(&*it);
//		}
//	}
//}
//
//void OcTreeLabel::updateNodeLogOdds(OcTreeNodeLabel* node, const float& update) const {
//	OccupancyOcTreeBase<OcTreeNodeLabel>::updateNodeLogOdds(node, update);
//	node->updateTimestamp();
//}
//
//void OcTreeLabel::integrateMissNoTime(OcTreeNodeLabel* node) const{
//	OccupancyOcTreeBase<OcTreeNodeLabel>::updateNodeLogOdds(node, prob_miss_log);
//}

OcTreeLabel::StaticMemberInitializer OcTreeLabel::ocTreeLabelMemberInit;

} // end namespace
