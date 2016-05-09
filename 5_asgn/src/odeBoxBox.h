#pragma once
#ifndef _ODEBOXBOX_
#define _ODEBOXBOX_

#include <Eigen/Dense>

//
// Contact structure representing ALL detected contact points
//
struct Contacts
{
	// Number of contacts
	int count;
	// Maximum penetration depth
	double depthMax;
	// Contact points in world space
	Eigen::Vector3d positions[8];
	// Contact normal (same for all contact points)
	Eigen::Vector3d normal;
};

//
// Main function to call
//
Contacts odeBoxBox(
	const Eigen::Matrix4d &M1, const Eigen::Vector3d &dimensions1,
	const Eigen::Matrix4d &M2, const Eigen::Vector3d &dimensions2);

#endif
