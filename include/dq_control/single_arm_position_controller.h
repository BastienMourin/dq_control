#ifndef SINGLE_ARM_POSITION_CONTROLLER_H
#define SINGLE_ARM_POSITION_CONTROLLER_H

#include <dq_operations/dq.h>
#include <dq_control/controller.h>
using namespace Eigen;

/**
 * @brief Class for Baxter arms. Can be left or right depending on constructor
 * 
 */
class SingleArmPositionController : public Controller
{
	private:
						
		/**
		 * error screw axes params
		 */
		RowVector3d l_e, m_e;
		/**
		 * error screw displacement dual angle params
		 */
		double theta_e, d_e;
		/**
		 * jacobian_8d
		 */
		MatrixXd jacobian_8d;
		/**
		 * gains for the screw error axis 
		 */
		double gain_real, gain_dual;
		/**
		 * damping parameter for damped pseudo-inverse
		 */
		double damping_param_pinv; 

		/**
		 * jacobian
		 */		
		MatrixXd jacobian;
		/**
		 * @brief Computed joint commands
		 */
		RowVectorXd joint_cmds;

		RowVectorXd ee_twist;
		/**
		 * proportional position and orientation gain  
		 */
		double k_p_p, k_p_o;		
		/**
		 * Current and desired pose
		 */
		Matrix<double,8,1> pose_current, pose_desired;	
		/**
		 * for exponential hyperbolic gains
		 */
		double exponential_gain_real, exponential_gain_dual;
		/**
		 * real and dual bounds for twist error
		 */
		double bound_real, bound_dual;

		Matrix<double,8,1> pose_error, pose_error_last;

	public:

		/**
		 * controller type
		 */
		enum controllerType {DQ_TWIST, SWITCHING_TWIST, BOUNDING_TWIST, HYPERBOLIC_TWIST, HYPERBOLIC_TWIST_2, HTM_BASED, DQ_ADORNO};		


		/**
		 * @brief set to the type of controller you want DQ_TWIST, SWITCHING_TWIST, BOUNDING_TWIST, HYPERBOLIC_TWIST, HYPERBOLIC_TWIST_2
		 * 
		 * @param controller_type_ 
		 */
		void setControllerSubType(controllerType controller_type_, double damping_param_pinv_ = 0.001);		



		/**
		 * @brief set exponential gains for HYPERBOLIC_TWIST_2 controller type		 * 
		 * @param k_p_p, k_p_o, bound_real, bound_dual, exponential_gain_real and exponential_gain_dual 
		 */
		void setGains(double k_p_p_, double k_p_o_, double bound_real_, double bound_dual_, double exponential_gain_real_, double exponential_gain_dual_);	
		/**
		 * @brief set real and dual terms of proportional gain, for DQ_TWIST and SWITCHING_TWIST controllers
		 * 
		 * @param k_p_p_ 
		 * @param k_p_o_ 
		 */
		void setGains(double k_p_p_, double k_p_o_);	

		/**
		 * @brief set exponential gains for BOUNDING_TWIST and HYPERBOLIC_TWIST controller type 
		 * @param k_p_p, k_p_o, bound_real, bound_dual
		 */
		void setGains(double k_p_p_, double k_p_o_, double bound_real_, double bound_dual_);	

		/**
		 * @brief get screw axis gains
		 * 
		 * @param gain_real_ 
		 * @param gain_dual_ 
		 */
		void getComputedErrorScrewGains(double &gain_real_, double &gain_dual_);
		/**
		 * @brief get screw axis l and m param
		 * 
		 * @param l_e_ 
		 * @param m_e_ 
		 */
		void getComputedErrorScrewAxes(RowVector3d &l_e_, RowVector3d &m_e_);
		/**
		 * @brief get error screw params
		 * 
		 * @param l_e_ 
		 * @param m_e_ 
		 * @param theta_e_ 
		 * @param d_e_ 
		 */
		void getComputedErrorScrewParams(RowVector3d &l_e_, RowVector3d &m_e_, double &theta_e_, double &d_e_);
		/**
		 * @brief get computed joint commands
		 * 
		 * @param joint_cmds 
		 */
		RowVectorXd getComputedJointCommands();
	
		/**
		 * @brief initializes with whatever needed
		 */

		RowVectorXd getComputedScrewError();

		void initializeController();
		/**
		 * @brief Compute Joint commands
		 */
		void updateController(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_, MatrixXd jacobian_);

        void updateControllerScrewError(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_);
		
		void updateDecoupledController(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_, RowVector3d position_error_, MatrixXd jacobian_);

		void updateDecoupledControllerScrewError(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_, RowVector3d position_error_);

		SingleArmPositionController();
		~SingleArmPositionController();

	private:
		controllerType controller_type;				
};

#endif // SINGLE_ARM_POSITION_CONTROLLER_H