//
// Created by kunhuang on 10/23/19.
//

#ifndef ARMLAB_WS_PROBVOXEL_H
#define ARMLAB_WS_PROBVOXEL_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ctime>

namespace octomap {

// node definition
class OcTreeNodeLabel : public OcTreeNode {

public:
	OcTreeNodeLabel() : OcTreeNode(), label(0) {}

	OcTreeNodeLabel(const OcTreeNodeLabel& rhs) : OcTreeNode(rhs), label(rhs.label) {}

	bool operator==(const OcTreeNodeLabel& rhs) const{
		return (rhs.value == value && rhs.label == label);
	}

	void copyData(const OcTreeNodeLabel& from){
		OcTreeNode::copyData(from);
		label = from.getLabel();
	}

	// label
	inline unsigned int getLabel() const { return label; }
//	inline void updateTimestamp() { label = (unsigned int) time(NULL);}
	inline void setTimestamp(unsigned int l) {label = l; }

	// update occupancy and label of inner nodes
//	inline void updateOccupancyChildren() {
//		this->setLogOdds(this->getMaxChildLogOdds());  // conservative
//		updateTimestamp();
//	}

protected:
	unsigned int label;
};


// tree definition
class OcTreeLabel : public OccupancyOcTreeBase<OcTreeNodeLabel> {

public:
	/// Default constructor, sets resolution of leafs
	OcTreeLabel(double resolution);

	/// virtual constructor: creates a new object of same type
	/// (Covariant return type requires an up-to-date compiler)
	OcTreeLabel* create() const {return new OcTreeLabel(resolution); }

	std::string getTreeType() const {return "OcTreeLabel";}

	//! \return label of last update
//	unsigned int getLastUpdateTime();

//	void degradeOutdatedNodes(unsigned int time_thres);

//	virtual void updateNodeLogOdds(OcTreeNodeLabel* node, const float& update) const;
//	void integrateMissNoTime(OcTreeNodeLabel* node) const;

protected:
	/**
	 * Static member object which ensures that this OcTree's prototype
	 * ends up in the classIDMapping only once. You need this as a
	 * static member in any derived octree class in order to read .ot
	 * files through the AbstractOcTree factory. You should also call
	 * ensureLinking() once from the constructor.
	 */
	class StaticMemberInitializer{
	public:
		StaticMemberInitializer() {
			OcTreeLabel* tree = new OcTreeLabel(0.1);
			tree->clearKeyRays();
			AbstractOcTree::registerTreeType(tree);
		}

		/**
		* Dummy function to ensure that MSVC does not drop the
		* StaticMemberInitializer, causing this tree failing to register.
		* Needs to be called from the constructor of this octree.
		*/
		void ensureLinking() {};
	};
	/// to ensure static initialization (only once)
	static StaticMemberInitializer ocTreeLabelMemberInit;

};

}
#endif //ARMLAB_WS_PROBVOXEL_H
