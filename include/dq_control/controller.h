#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <dq_operations/dq.h>
using namespace Eigen;

/**
 * @brief Class for Baxter arms. Can be left or right depending on constructor
 * 
 */
class Controller
{
	public:
		virtual void initializeController(){};
		virtual void updateController(){};
		Controller();
		~Controller();
};
#endif