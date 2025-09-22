#include <dq_control/single_arm_position_controller.h>
using namespace Eigen;

SingleArmPositionController::SingleArmPositionController(){}
SingleArmPositionController::~SingleArmPositionController(){}

void SingleArmPositionController::setControllerSubType(controllerType controller_type_, double damping_param_pinv_)
{
	controller_type = controller_type_;
	damping_param_pinv = damping_param_pinv_;
}	

void SingleArmPositionController::setGains(double k_p_o_, double k_p_p_)
{
	k_p_p = k_p_p_; 
	k_p_o = k_p_o_;	
}	
void SingleArmPositionController::setGains(double k_p_o_, double k_p_p_, double bound_real_, double bound_dual_)
{
	k_p_p = k_p_p_; 
	k_p_o = k_p_o_;		
	bound_real = bound_real_; 
	bound_dual = bound_dual_;
}	
void SingleArmPositionController::setGains(double k_p_o_, double k_p_p_, double bound_real_, double bound_dual_, double exponential_gain_real_, double exponential_gain_dual_)
{
	k_p_p = k_p_p_; 
	k_p_o = k_p_o_;		
	bound_real = bound_real_; 
	bound_dual = bound_dual_;	
	exponential_gain_real = exponential_gain_real_;
	exponential_gain_dual = exponential_gain_dual_;
}

void SingleArmPositionController::getComputedErrorScrewGains(double &gain_real_, double &gain_dual_)
{
	gain_real_ = gain_real; 
	gain_dual_ = gain_dual;
}
void SingleArmPositionController::getComputedErrorScrewAxes(RowVector3d &l_e_, RowVector3d &m_e_)
{
	l_e_ = l_e; 
	m_e_ = m_e;
}

void SingleArmPositionController::getComputedErrorScrewParams(RowVector3d &l_e_, RowVector3d &m_e_, double &theta_e_, double &d_e_)
{
	l_e_ = l_e; 
	m_e_ = m_e;
	theta_e_ = theta_e;
	d_e_ = d_e;
}


void SingleArmPositionController::initializeController()
{
	pose_error_last << 1, 0, 0, 0, 0, 0, 0, 0;
	damping_param_pinv=0.001;
	controller_type = DQ_TWIST;
	gain_real=0;
	gain_dual=0;
	bound_real=0;
	bound_dual=0;
	theta_e=0;
	d_e=0;
	exponential_gain_real=0;
	exponential_gain_dual=0;
	k_p_p=0;
	k_p_o=0;
	exponential_gain_dual=0;
	l_e << 0.0, 0.0, 0.0;
	m_e << 0.0, 0.0, 0.0;
}

void SingleArmPositionController::updateController(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_, MatrixXd jacobian_)
{
	jacobian = jacobian_;
	pose_desired = pose_desired_;
	pose_current = pose_current_;
	int joint_size = jacobian.cols();
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	if (theta_e > M_PI)
	{
		pose_desired=-pose_desired;
	}
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	
    // theta_e= DQ::normalizeAngleZeroTo2Pi(theta_e);  
	RowVectorXd screw_error = RowVectorXd::Zero(6);

	if (controller_type == DQ_TWIST)
	{
		gain_real = k_p_o*theta_e;
		gain_dual = k_p_p*d_e;
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}
	else if (controller_type == SWITCHING_TWIST)
	{
		gain_real = k_p_o;
		gain_dual = k_p_p;				
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}
	else if (controller_type == BOUNDING_TWIST)
	{
	    gain_real = k_p_o*theta_e;
	    gain_dual = k_p_p*d_e;
	    //------------------
	    RowVector3d trans_bound;
	    trans_bound = gain_real *m_e + gain_dual *l_e;
		double norm_trans_bound = trans_bound.norm();
		RowVector3d trans_dir;
		trans_dir << 0.0, 0.0, 0.0;
		if (norm_trans_bound!=0)
			trans_dir = trans_bound/norm_trans_bound;

		//------------------
	    /*
	    if (fabs(gain_real) > bound_real)
	      gain_real = bound_real*gain_real/fabs(gain_real);
	    if (fabs(gain_dual) > bound_dual)
	      gain_dual = bound_dual*gain_dual/fabs(gain_dual);
	      */
		//------------------
		// if (fabs(gain_real) > bound_real)
	 //      gain_real = bound_real;//*gain_real/fabs(gain_real);
	    if (fabs(norm_trans_bound) > bound_dual)
	    {
	      norm_trans_bound = bound_dual;//*norm_trans_bound/fabs(norm_trans_bound);
	      // l.m/norm(m)^2
	    }

	    RowVector3d Vs;
	    Vs = norm_trans_bound*trans_dir;

	    if (m_e.norm() != 0)
	    {
	    	gain_real = (Vs(0)*m_e(0) + Vs(1)*m_e(1) + Vs(2)*m_e(2))/pow(m_e.norm(),2); 
	    }else {
	    	gain_real = k_p_o*theta_e;
	    	if (fabs(gain_real) > bound_real)
	        gain_real = bound_real*gain_real/fabs(gain_real);
	    }

	    
		screw_error << gain_real *l_e, Vs;
	}  	
	else if (controller_type == HYPERBOLIC_TWIST)
	{
		gain_real = k_p_o*theta_e;
	    gain_dual = k_p_p*d_e;
	    //------------------
	    RowVector3d trans_bound;
	    trans_bound = gain_real *m_e + gain_dual *l_e;
		double norm_trans_bound = trans_bound.norm();
		RowVector3d trans_dir;
		trans_dir << 0.0, 0.0, 0.0;
		if (norm_trans_bound!=0)
			trans_dir = trans_bound/norm_trans_bound;

		//------------------
	    /*
	    if (fabs(gain_real) > bound_real)
	      gain_real = bound_real*gain_real/fabs(gain_real);
	    if (fabs(gain_dual) > bound_dual)
	      gain_dual = bound_dual*gain_dual/fabs(gain_dual);
	      */
		//------------------
		// if (fabs(gain_real) > bound_real)
	 //      gain_real = bound_real;//*gain_real/fabs(gain_real);
	    if (fabs(norm_trans_bound) > bound_dual)
	    {
	      norm_trans_bound = bound_dual;//*norm_trans_bound/fabs(norm_trans_bound);
	      // l.m/norm(m)^2
	    }

	    RowVector3d Vs;
	    Vs = norm_trans_bound*trans_dir;

		double sat_real, sat_dual;
	    if (m_e.norm() != 0)
	    {
	    	sat_real = (Vs(0)*m_e(0) + Vs(1)*m_e(1) + Vs(2)*m_e(2))/pow(m_e.norm(),2); 
	    }else {
	    	sat_real = bound_real;
	    }

	    if (l_e.norm() != 0)
	    {
	    	sat_dual = (Vs(0)*l_e(0) + Vs(1)*l_e(1) + Vs(2)*l_e(2))/pow(l_e.norm(),2); 
	    }else {
	    	sat_dual = bound_dual;
	    }


		gain_real = sat_real*tanh((k_p_o/sat_real)*theta_e);
		gain_dual = sat_dual*tanh((k_p_p/sat_dual)*d_e);			  
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}  
	else if (controller_type == HYPERBOLIC_TWIST_2)
	{
		gain_real = bound_real*(1-std::exp(-exponential_gain_real*theta_e*theta_e))*tanh(k_p_o*theta_e/bound_real);
		gain_dual = bound_dual*(1-std::exp(-exponential_gain_dual*d_e*d_e))*tanh(k_p_p*d_e/bound_dual);				
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}  				
	// screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		

	MatrixXd jacobian_damped_, A;
	A= MatrixXd::Zero(joint_size, joint_size);
	jacobian_damped_= MatrixXd::Zero(joint_size, 6);
	A = jacobian.transpose() * jacobian + damping_param_pinv * MatrixXd::Identity(joint_size, joint_size);
	// std::cout  << A << std::endl;		
	jacobian_damped_ = ((A.inverse()) * jacobian.transpose());
	// screw_error(0) = 0.0;
	// screw_error(1) = 0.0;
	// screw_error(2) = 0.0;
	joint_cmds = (jacobian_damped_ * screw_error.transpose()).transpose();
	ee_twist = screw_error;
	pose_error_last = pose_error;
};

void SingleArmPositionController::updateControllerScrewError(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_)
{
	pose_desired = pose_desired_;
	pose_current = pose_current_;
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	if (theta_e > M_PI)
	{
		pose_desired=-pose_desired;
	}
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	

    // theta_e= DQ::normalizeAngleZeroTo2Pi(theta_e);  
	RowVectorXd screw_error = RowVectorXd::Zero(6);

	if (controller_type == DQ_TWIST)
	{
		std::cout << "Applying DQ twist controller";
		gain_real = k_p_o*theta_e;
		gain_dual = k_p_p*d_e;
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}
	else if (controller_type == SWITCHING_TWIST)
	{
		gain_real = k_p_o;
		gain_dual = k_p_p;				
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}
	else if (controller_type == BOUNDING_TWIST)
	{
		std::cout << "Applying bounding twist controller";
	    gain_real = k_p_o*theta_e;
	    gain_dual = k_p_p*d_e;
	    //------------------
	    RowVector3d trans_bound;
	    trans_bound = gain_real *m_e + gain_dual *l_e;
		double norm_trans_bound = trans_bound.norm();
		RowVector3d trans_dir;
		trans_dir << 0.0, 0.0, 0.0;
		if (norm_trans_bound!=0)
			trans_dir = trans_bound/norm_trans_bound;

		//------------------
	    /*
	    if (fabs(gain_real) > bound_real)
	      gain_real = bound_real*gain_real/fabs(gain_real);
	    if (fabs(gain_dual) > bound_dual)
	      gain_dual = bound_dual*gain_dual/fabs(gain_dual);
	      */
		//------------------
		// if (fabs(gain_real) > bound_real)
	 //      gain_real = bound_real;//*gain_real/fabs(gain_real);
	    if (fabs(norm_trans_bound) > bound_dual)
	    {
	      norm_trans_bound = bound_dual;//*norm_trans_bound/fabs(norm_trans_bound);
	      // l.m/norm(m)^2
	    }

	    RowVector3d Vs;
	    Vs = norm_trans_bound*trans_dir;

	    if (m_e.norm() != 0)
	    {
	    	gain_real = (Vs(0)*m_e(0) + Vs(1)*m_e(1) + Vs(2)*m_e(2))/pow(m_e.norm(),2); 
	    }else {
	    	gain_real = k_p_o*theta_e;
	    	if (fabs(gain_real) > bound_real)
	        gain_real = bound_real*gain_real/fabs(gain_real);
	    }

	    
		screw_error << gain_real *l_e, Vs;
	}  	
	else if (controller_type == HYPERBOLIC_TWIST)
	{
		gain_real = k_p_o*theta_e;
	    gain_dual = k_p_p*d_e;
	    //------------------
	    RowVector3d trans_bound;
	    trans_bound = gain_real *m_e + gain_dual *l_e;
		double norm_trans_bound = trans_bound.norm();
		RowVector3d trans_dir;
		trans_dir << 0.0, 0.0, 0.0;
		if (norm_trans_bound!=0)
			trans_dir = trans_bound/norm_trans_bound;

		//------------------
	    /*
	    if (fabs(gain_real) > bound_real)
	      gain_real = bound_real*gain_real/fabs(gain_real);
	    if (fabs(gain_dual) > bound_dual)
	      gain_dual = bound_dual*gain_dual/fabs(gain_dual);
	      */
		//------------------
		// if (fabs(gain_real) > bound_real)
	 //      gain_real = bound_real;//*gain_real/fabs(gain_real);
	    if (fabs(norm_trans_bound) > bound_dual)
	    {
	      norm_trans_bound = bound_dual;//*norm_trans_bound/fabs(norm_trans_bound);
	      // l.m/norm(m)^2
	    }

	    RowVector3d Vs;
	    Vs = norm_trans_bound*trans_dir;

		double sat_real, sat_dual;
	    if (m_e.norm() != 0)
	    {
	    	sat_real = (Vs(0)*m_e(0) + Vs(1)*m_e(1) + Vs(2)*m_e(2))/pow(m_e.norm(),2); 
	    }else {
	    	sat_real = bound_real;
	    }

	    if (l_e.norm() != 0)
	    {
	    	sat_dual = (Vs(0)*l_e(0) + Vs(1)*l_e(1) + Vs(2)*l_e(2))/pow(l_e.norm(),2); 
	    }else {
	    	sat_dual = bound_dual;
	    }


		gain_real = sat_real*tanh((k_p_o/sat_real)*theta_e);
		gain_dual = sat_dual*tanh((k_p_p/sat_dual)*d_e);			  
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}  
	else if (controller_type == HYPERBOLIC_TWIST_2)
	{
		gain_real = bound_real*(1-std::exp(-exponential_gain_real*theta_e*theta_e))*tanh(k_p_o*theta_e/bound_real);
		gain_dual = bound_dual*(1-std::exp(-exponential_gain_dual*d_e*d_e))*tanh(k_p_p*d_e/bound_dual);				
		screw_error << gain_real *l_e, gain_real *m_e + gain_dual *l_e;		
	}  				
	
	ee_twist = screw_error;
	pose_error_last = pose_error;
};


void SingleArmPositionController::updateDecoupledController(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_, RowVector3d position_error_, MatrixXd jacobian_)
{
	jacobian = jacobian_;
	pose_desired = pose_desired_;
	pose_current = pose_current_;
	int joint_size = jacobian.cols();
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	if (theta_e > M_PI)
	{
		pose_desired=-pose_desired;
	}
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	
    // theta_e= DQ::normalizeAngleZeroTo2Pi(theta_e);  
	RowVectorXd screw_error = RowVectorXd::Zero(6);

	screw_error << k_p_o*theta_e *l_e, k_p_p * position_error_;		
	
	MatrixXd jacobian_damped_, A;
	A= MatrixXd::Zero(joint_size, joint_size);
	jacobian_damped_= MatrixXd::Zero(joint_size, 6);
	A = jacobian.transpose() * jacobian + damping_param_pinv * MatrixXd::Identity(joint_size, joint_size);
	// std::cout  << A << std::endl;		
	jacobian_damped_ = ((A.inverse()) * jacobian.transpose());
	// screw_error(0) = 0.0;
	// screw_error(1) = 0.0;
	// screw_error(2) = 0.0;
	joint_cmds = (jacobian_damped_ * screw_error.transpose()).transpose();
	ee_twist = screw_error;
	pose_error_last = pose_error;
};

void SingleArmPositionController::updateDecoupledControllerScrewError(Matrix<double,8,1> pose_current_, Matrix<double,8,1> pose_desired_, RowVector3d position_error_)
{
	pose_desired = pose_desired_;
	pose_current = pose_current_;
	int joint_size = jacobian.cols();
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	if (theta_e > M_PI)
	{
		pose_desired=-pose_desired;
	}
	pose_error=DQ::multiplyDQ(pose_desired, DQ::classicConjugateDQ(pose_current));
	DQ::dqToScrewParameters(pose_error, theta_e, d_e, l_e, m_e);
	
    // theta_e= DQ::normalizeAngleZeroTo2Pi(theta_e);  
	RowVectorXd screw_error = RowVectorXd::Zero(6);

	screw_error << k_p_o*theta_e *l_e, k_p_p * position_error_;		
	
	ee_twist = screw_error;
	pose_error_last = pose_error;
};


RowVectorXd SingleArmPositionController::getComputedJointCommands()
{
	return joint_cmds;
}

RowVectorXd SingleArmPositionController::getComputedScrewError()
{
	return ee_twist;
}
