//
// Created by arprice on 12/19/18.
//

#ifndef PROJECT_OBJECTINDEX_H
#define PROJECT_OBJECTINDEX_H

struct ObjectIndex
{
	long id = -1;
	bool operator<(const ObjectIndex& other) const { return id < other.id; }
};

#endif //PROJECT_OBJECTINDEX_H
