#pragma once


#include <Eigen/Eigen>

//////////////////////////////

class RigidBody
{

public:
	RigidBody()
    {
    };

	~RigidBody() 
    {
    };

    

public:

    float dt_ = 0.001;
    float m_elapsed_time_scale = 1.0;
    bool is_contact = false;
    bool is_free_motion = true;
    Eigen::Vector3f m_contact_pt;
    Eigen::Vector3f m_contact_normal;
    Eigen::Vector3f m_r_vector_com2contact;
    Eigen::MatrixXf m_contact_pt_set;
    Eigen::MatrixXf m_contact_normal_set;
    Eigen::MatrixXf m_r_vector_com2contact_set;
    float m_impulse_intensity = 0.0;


	Eigen::Matrix3f m_InertiaTensorWorld;
	Eigen::Matrix3f m_invInertiaTensorWorld;
	Eigen::Vector3f m_localInertia;

	Eigen::Vector3f m_linearPosition;
	Eigen::Vector3f m_linearVelocity;
	Eigen::Vector3f m_linearVelocityImpulse;
	Eigen::Vector3f m_linearVelocityGravity;
	Eigen::Vector3f m_prev_linearVelocity;
    Eigen::Vector3f m_angularVelocity;
    Eigen::Vector3f m_angularVelocityImpulse;
	Eigen::Vector3f m_angularVelocityGravity;
	Eigen::Vector3f m_prev_angularVelocity;

	// Eigen::Vector3f m_angularPosition;
	Eigen::Matrix3f m_R_orientation;
    Eigen::Quaternionf m_quaternion_orientation;

	float m_mass;
	float m_inverseMass;
	Eigen::Vector3f m_linearFactor;
	Eigen::Vector3f m_impulseLinearFactor;
    

	Eigen::Vector3f m_gravity;
	Eigen::Vector3f m_gravity_acceleration;
	Eigen::Vector3f m_invInertiaLocal;
	Eigen::Vector3f m_totalForce;
	Eigen::Vector3f m_totalTorque;

	float m_linearDamping;
	float m_angularDamping;

	bool m_additionalDamping;
	float m_additionalDampingFactor;
	float m_additionalLinearDampingThresholdSqr;
	float m_additionalAngularDampingThresholdSqr;
	float m_additionalAngularDampingFactor;

	float m_linearSleepingThreshold;
	float m_angularSleepingThreshold;


	int m_RigidBodyFlags;

	int m_debugBodyId;

protected:
	Eigen::Vector3f m_deltaAngularVelocity;
	Eigen::Vector3f m_angularFactor;
	Eigen::Vector3f m_impulseAngularFactor;
    
	Eigen::Vector3f m_pushVelocity;
	Eigen::Vector3f m_turnVelocity;


public:

    void setPosition(const Eigen::Vector3f &linear_position, const Eigen::Matrix3f &R)
    {
        m_linearPosition = linear_position;
        m_R_orientation = R;
        Eigen::Quaternionf q(R);
        // std::cout << "(before) m_quaternion_orientation " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
        q.normalize();
        // std::cout << "(after) m_quaternion_orientation " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
        m_quaternion_orientation = q;
    }

    Eigen::Matrix4f getPosition() 
    { 
        Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
        output.block<3, 1>(0, 3) = m_linearPosition;
        output.block<3, 3>(0, 0) = m_R_orientation;
        return output;
    }

    Eigen::VectorXf getVelocity() 
    { 
        Eigen::VectorXf output(6);
        output.segment<3>(0) = m_linearVelocity;
        output.segment<3>(3) = m_angularVelocity;
        return output;
    }

    Eigen::VectorXf getForce() 
    { 
        Eigen::VectorXf output(6);
        output.segment<3>(0) = m_totalForce;
        output.segment<3>(3) = m_totalTorque;
        return output;
    }

    void initializeLinearVelocity()
    {
        m_prev_linearVelocity = m_linearVelocity;
        m_linearVelocity = Eigen::Vector3f::Zero();
    }

    void initializeAngularVelocity()
    {
        m_prev_angularVelocity = m_angularVelocity;
        m_angularVelocity = Eigen::Vector3f::Zero();
    }

    void setControlPeriod(const float &dt)
    {
        dt_ = dt;
    }

    void setElapsedTimeScale(const float &elapsed_time_scale)
    {
        m_elapsed_time_scale = elapsed_time_scale;
    }

	void step()
	{

        // F_c
        if(is_contact) 
        {
            //// TODO: 작용-반작용에 따른 반력 적용하여 중력 방향으로 계속 가속되는 것을 방지
            // applyRepulsiveForce();

            //// TODO: 접촉 순간에는 impulse에 해당하는 속도, 가속도만 존재하므로... 속도 초기화 필요?
            //// TODO: 추후 두 물체 사이의 운동량 보존 법칙 적용하기

            if(is_free_motion)
            {
                initializeLinearVelocity();
                initializeAngularVelocity();
                is_free_motion = false;
            }
            // computeInvInertiaTensorWorld(); // base frame 기준으로 MOI를 변환

            // unit impulse에 따른 속도 각속도 할당
            if(0) resolveCollision(); // 접촉 점군의 중심점 이용
            else resolveCollisionSet(); // 접촉 점군의 모든 점 이용
            
        }
        else
        {
            // F_ext
            // m_totalForce += m_mass * m_gravity_acceleration; // apply a force
            applyCentralForce(m_mass * m_gravity_acceleration);
        }

        integrateVelocity();



	}
    
    Eigen::Quaternionf omega_to_qdot(const Eigen::Quaternionf &q, const Eigen::Vector3f &omega) 
    {
        Eigen::Quaternionf output;
        Eigen::MatrixXf m(4, 3);
        // m(0, 0) =  q.w();   m(0, 1) = -q.z();   m(0, 2) =  q.y();
        // m(1, 0) =  q.z();   m(1, 1) =  q.w();   m(1, 2) = -q.x();
        // m(2, 0) = -q.y();   m(2, 1) =  q.x();   m(2, 2) =  q.w();
        // m(3, 0) = -q.x();   m(3, 1) = -q.y();   m(3, 2) = -q.z();
        m(0, 0) =  q.w();   m(0, 1) =  q.z();   m(0, 2) = -q.y();
        m(1, 0) = -q.z();   m(1, 1) =  q.w();   m(1, 2) =  q.x();
        m(2, 0) =  q.y();   m(2, 1) = -q.x();   m(2, 2) =  q.w();
        m(3, 0) = -q.x();   m(3, 1) = -q.y();   m(3, 2) = -q.z();

        Eigen::Vector4f tmp = 0.5 * m * omega;
        output.x() = tmp[0];
        output.y() = tmp[1];
        output.z() = tmp[2];
        output.w() = tmp[3];
        return output;
        // return Quaternion(0.5 * m * omega);
    }

    Eigen::Quaternionf quaternion_multiplication(const Eigen::Quaternionf &input, const Eigen::Quaternionf &q)
    { // 0, 1, 2: x, y,z / 3: w
        Eigen::Quaternionf output;
        output.x() = input.w() * q.x() + input.x() * q.w() + input.y() * q.z() - input.z() * q.y();
        output.y() = input.w() * q.y() + input.y() * q.w() + input.z() * q.x() - input.x() * q.z();
        output.z() = input.w() * q.z() + input.z() * q.w() + input.x() * q.y() - input.y() * q.x();
        output.w() = input.w() * q.w() - input.x() * q.x() - input.y() * q.y() - input.z() * q.z();

        return output;
    }

    void integrateVelocity()
    {
        //////////////////////////////////////////////////////
        // ////////////////// applyForce()로 이동 X ////////////////
        // //// NOTICE: 최종 속도, 각속도를 이용하여 여기서 결정해야 함. --> 속도 중복되어 잘못된 값 추출됨
        // //////////////////////////////////////////////////////
        // //// Simpletic euler method
        // // m_linearVelocity += m_totalForce / m_mass * dt_;
        // m_linearVelocity += m_totalForce / m_mass * dt_ * m_elapsed_time_scale; // m_elapsed_time_scale [ms] :  연산시간에 따른 지연 보상
        
        // // 중력에 의한 COM에서의 각속도 적용?
		// // m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * dt_;
		// m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * m_elapsed_time_scale; // m_elapsed_time_scale [ms] :  연산시간에 따른 지연 보상
 
        // //// TODO: 추후 각속도에 의한 선속도 적용 여부 결정
        // // m_linearVelocity += m_angularVelocity.cross(m_r_vector_com2contact);
        //////////////////////////////////////////////////////


        m_linearPosition += m_linearVelocity * dt_;

        Eigen::Quaternionf q_tmp = omega_to_qdot(m_quaternion_orientation, m_angularVelocity);
        // q_tmp.x() *= dt_;
        // q_tmp.y() *= dt_;
        // q_tmp.z() *= dt_;
        // q_tmp.w() *= dt_;

        //// TODO: Quaternion summation
        m_quaternion_orientation.x() += q_tmp.x();
        m_quaternion_orientation.y() += q_tmp.y();
        m_quaternion_orientation.z() += q_tmp.z();
        m_quaternion_orientation.w() += q_tmp.w();
        m_quaternion_orientation.normalize();

        Eigen::Quaternionf q = m_quaternion_orientation;
        // std::cout << "(final) m_quaternion_orientation " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;

        m_R_orientation = m_quaternion_orientation.toRotationMatrix();

        computeInvInertiaTensorWorld(); // base frame 기준으로 MOI를 변환

		// }

    }

	void computeInvInertiaTensorWorld()
	{

        // btMatrix3x3 btMatrix3x3::scaled	(	const btVector3 & 	s	)	const
        // s: Scaling vector The elements of the vector will scale each column
		// m_invInertiaTensorWorld = m.scaled(invInertiaLocal) * m.transpose();
        Eigen::Matrix3f m = m_R_orientation;
        // m_invInertiaTensorWorld = m.scaled(invInertiaLocal) * m.transpose();
        // std::cout << "m: " << m << std::endl;
        // std::cout << "m_localInertia: " << m_localInertia[0] << ", " << m_localInertia[1] << ", " << m_localInertia[2] << "" << std::endl;
        Eigen::Matrix3f tmp = m.array().rowwise() *m_localInertia.transpose().array();
        // std::cout << "tmp: " << tmp << std::endl;
        m_invInertiaTensorWorld = tmp * m.transpose();
	}

    void applyRepulsiveForce()
    {
        //// Constraint collision solver
        float applied_impulse = 0.0;
        // Impulse = F*delta_t = mass*delta_v, Impulse_torque = tau*delta_t = moment of inertia*delta_omega
        Eigen::Vector3f rel_pos1 = m_r_vector_com2contact;
        applyForce(-m_totalForce, rel_pos1);
        printf("\n\nm_totalForce(after repulsive force): %0.4f, %0.4f, %0.4f\n\n", m_totalForce[0], m_totalForce[1], m_totalForce[2]);
        printf("\n\nm_totalTorque(after repulsive force): %0.4f, %0.4f, %0.4f\n\n", m_totalTorque[0], m_totalTorque[1], m_totalTorque[2]);
    }

    void resolveCollision()
    {
        // //// Constraint collision solver
        // float applied_impulse = 0.0;
        // // Impulse = F*delta_t = mass*delta_v, Impulse_torque = tau*delta_t = moment of inertia*delta_omega
        // Eigen::Vector3f rel_pos1 = m_r_vector_com2contact;
        // // Eigen::Vector3f rel_pos2(0.0, 0.0, 0.0);
        // // Eigen::Vector3f vel_bodyA = m_linearVelocity;
        // Eigen::Vector3f vel_bodyA = getVelocityInLocalPoint(rel_pos1);
        // Eigen::Vector3f vel_bodyB(0.0, 0.0, 0.0);
        // float rel_vel = m_contact_normal.dot(vel_bodyA - vel_bodyB);

        // Eigen::Vector3f temp1 = m_invInertiaTensorWorld*rel_pos1.cross(m_contact_normal);
        // // Eigen::Vector3f temp2 = inertia_matrix_B.inverse()*rel_pos2.cross(m_contact_normal);
        // float gRestitution = 0.0;
        // float impulse = -(m_impulse_intensity + gRestitution)*rel_vel/(m_inverseMass + m_contact_normal.dot(temp1.cross(rel_pos1)));
        // Eigen::Vector3f impulse_vector = m_contact_normal*impulse;
        
        // printf("impulse_vector(impulse: %0.4f): %0.4f, %0.4f, %0.4f\n", impulse, impulse_vector[0], impulse_vector[1], impulse_vector[2]);
        // // applied_impulse = impulse;

        // applyImpulse(impulse_vector, rel_pos1);
        // // Apply force?
        // // applyForce(m_totalForce, rel_pos1);
    }

    void resolveCollisionSet()
    {
        m_linearVelocityGravity = Eigen::Vector3f::Zero();
        m_angularVelocityGravity = Eigen::Vector3f::Zero();
        m_linearVelocityImpulse = Eigen::Vector3f::Zero();
        m_angularVelocityImpulse = Eigen::Vector3f::Zero();
        //// Constraint collision solver
        float applied_impulse = 0.0;
        // Impulse = F*delta_t = mass*delta_v, Impulse_torque = tau*delta_t = moment of inertia*delta_omega
        float coeff_tmp = static_cast<float>(m_r_vector_com2contact_set.rows());

        // Apply gravity force
        Eigen::Vector3f applied_force = m_mass * m_gravity_acceleration;
        applied_force /= coeff_tmp;
        for (size_t i = 0; i < m_r_vector_com2contact_set.rows(); i++)
        {
            Eigen::Vector3f r_vector_com2contact = m_r_vector_com2contact_set.row(i);
            Eigen::Vector3f r_vector_contact2com = -r_vector_com2contact;

            // Eigen::Vector3f rel_pos2(0.0, 0.0, 0.0);
            // Eigen::Vector3f vel_bodyA = m_linearVelocity;
            // Eigen::Vector3f vel_bodyA = getVelocityInLocalPoint(r_vector_com2contact);
            Eigen::Vector3f vel_bodyA = getVelocityInLocalPointBeforeCollision(r_vector_com2contact);
            Eigen::Vector3f vel_bodyB(0.0, 0.0, 0.0);
            // printf("vel_bodyA: %0.4f, %0.4f, %0.4f\n", vel_bodyA[0], vel_bodyA[1], vel_bodyA[2]);


            // m_contact_normal: 물체 A에서 외부를 향하는 방향벡터 (물체 A 입장에서 접촉 법선의 반대 방향이 impulse vector의 방향벡터)
            Eigen::Vector3f contact_normal = m_contact_normal_set.row(i);
            // printf("contact_normal: %0.4f, %0.4f, %0.4f\n", contact_normal[0], contact_normal[1], contact_normal[2]);
            // printf("r_vector_com2contact: %0.4f, %0.4f, %0.4f\n", r_vector_com2contact[0], r_vector_com2contact[1], r_vector_com2contact[2]);
            float rel_vel = contact_normal.dot(vel_bodyA - vel_bodyB);

            Eigen::Vector3f temp1 = m_invInertiaTensorWorld*r_vector_com2contact.cross(contact_normal);
            // Eigen::Vector3f temp2 = inertia_matrix_B.inverse()*rel_pos2.cross(m_contact_normal);
            float gRestitution = 0.0;
            float impulse = -(m_impulse_intensity + gRestitution)*rel_vel/(m_inverseMass + contact_normal.dot(temp1.cross(r_vector_com2contact)));
            Eigen::Vector3f impulse_vector = contact_normal*impulse;

            if(1) impulse_vector /= coeff_tmp;
            
            // printf("impulse_vector(impulse: %0.4f): %0.4f, %0.4f, %0.4f\n", impulse, impulse_vector[0], impulse_vector[1], impulse_vector[2]);
            applied_impulse += impulse;

            applyImpulse(impulse_vector, r_vector_com2contact);




            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            //// CODEHERE: 접촉력에 따른 토크 계산 및 토크에 따른 각속도 변화 산출
            //// NOTICE: 접촉 시에만 COM에서 접촉점을 향하는 변위벡터를 가지므로 이 부분에서 applyForce()를 수행
            //// NOTICE: free motion 시에는 단순히 applyCentralForce()만 수행


            //// NOTICE: 여기서 적용되는 외력은 중력이고 중력은 무게중심에 작용함. 이때 회전중심은 접촉점이므로 모멘트 팔의 방향벡터는 접촉점으로부터 무게중심을 향하는 벡터
            applyForce(applied_force, r_vector_contact2com);
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////


        }
        //// Monitor
        
        printf("\n**********************************\n");
        printf("applied_impulse(impulse_intensity: %f): %f\n", m_impulse_intensity, applied_impulse);
        printf("m_inverseMass: %f\n", m_inverseMass);
        printf("m_localInertia(Ixx, Iyy, Izz): %f, %f, %f\n", m_localInertia[0], m_localInertia[1], m_localInertia[2]);
        printf("m_invInertiaTensorWorld: \n");
        std::cout << m_invInertiaTensorWorld << std::endl;

        printf("**********************************\n");
        printf("gravity_linearVelocity_Sum: %f, %f, %f\n", m_linearVelocityGravity[0], m_linearVelocityGravity[1], m_linearVelocityGravity[2]);
        printf("gravity_angularVelocity_Sum: %f, %f, %f\n", m_angularVelocityGravity[0], m_angularVelocityGravity[1], m_angularVelocityGravity[2]);
        printf("**********************************\n");
        printf("impulse_linearVelocity_Sum: %f, %f, %f\n", m_linearVelocityImpulse[0], m_linearVelocityImpulse[1], m_linearVelocityImpulse[2]);
        printf("impulse_angularVelocity_Sum: %f, %f, %f\n", m_angularVelocityImpulse[0], m_angularVelocityImpulse[1], m_angularVelocityImpulse[2]);
        printf("**********************************\n");

    }

    void setImpulseScale(float scale)
    {
        m_impulse_intensity = scale;
    }

    void setContact(bool flag)
    {
        is_contact = flag;
        if(!is_contact) is_free_motion = true;
    }

    void setContactPoint(const Eigen::Vector3f& contact_pt)
    {
        m_contact_pt = 0.001*contact_pt; // [m]
        // printf("m_contact_pt: %0.4f, %0.4f, %0.4f\n", m_contact_pt[0], m_contact_pt[1], m_contact_pt[2]);
    }

    void setContactNormal(const Eigen::Vector3f& contact_normal)
    {
        m_contact_normal = contact_normal; 
        // printf("m_contact_normal: %0.4f, %0.4f, %0.4f\n", m_contact_normal[0], m_contact_normal[1], m_contact_normal[2]);
    }

    void setRelativePosition(const Eigen::Vector3f& r_vector)
    {
        // printf("r_vector: %0.4f, %0.4f, %0.4f\n", r_vector[0], r_vector[1], r_vector[2]);
        m_r_vector_com2contact = 0.001*r_vector; // [m]
        // printf("m_r_vector_com2contact: %0.4f, %0.4f, %0.4f\n", m_r_vector_com2contact[0], m_r_vector_com2contact[1], m_r_vector_com2contact[2]);
    }

    void setContactPointSet(const Eigen::MatrixXf& contact_pt_set)
    {
        m_contact_pt_set = 0.001*contact_pt_set; // [m]
    }

    void setContactNormalSet(const Eigen::MatrixXf& contact_normal_set)
    {
        m_contact_normal_set = contact_normal_set; 
    }

    void setRelativePositionSet(const Eigen::MatrixXf& r_vector_set)
    {
        m_r_vector_com2contact_set = 0.001*r_vector_set; // [m]
    }
	
    void setGravity(const Eigen::Vector3f& acceleration)
    {
        m_gravity_acceleration = acceleration;
    }

	const Eigen::Vector3f& getGravity() const
	{
		return m_gravity_acceleration;
	}

	void setMassProps(float mass, const Eigen::Matrix3f& inertia)
    {
        m_mass = mass;
        m_inverseMass = float(1.0) / mass;

        m_localInertia[0] = inertia(0, 0);
        m_localInertia[1] = inertia(1, 1);
        m_localInertia[2] = inertia(2, 2);
        // m_InertiaTensorWorld = inertia;
        // m_invInertiaTensorWorld = inertia.inverse();
    }

	const Eigen::Vector3f& getLinearFactor() const
	{
		return m_linearFactor;
	}
    
	void setLinearFactor(const Eigen::Vector3f& linearFactor)
	{
		m_linearFactor = linearFactor;
	}
	
	void setImpulseLinearFactor(const Eigen::Vector3f& linearFactor)
	{
		m_impulseLinearFactor = linearFactor;
	}

    float getInvMass() const { return m_inverseMass; }
	
    float getMass() const { return m_inverseMass == float(0.) ? float(0.) : float(1.0) / m_inverseMass; }
	
    const Eigen::Matrix3f& getInvInertiaTensorWorld() const
	{
		return m_invInertiaTensorWorld;
	}

	void applyCentralForce(const Eigen::Vector3f& force)
	{
		// m_totalForce += force.cwiseProduct(m_linearFactor);
		Eigen::Vector3f force_tmp = force.cwiseProduct(m_linearFactor);
        m_totalForce += force_tmp;

        // ////////////////////////////////////////////////////////////////
        // //// Simpletic euler method
        // m_linearVelocity += m_totalForce / m_mass * dt_;
        // m_linearVelocity += m_totalForce / m_mass * dt_ * m_elapsed_time_scale; // m_elapsed_time_scale [ms] :  연산시간에 따른 지연 보상
        Eigen::Vector3f linearVelocity = force_tmp / m_mass * dt_ * m_elapsed_time_scale;
        m_linearVelocity += linearVelocity;
		m_linearVelocityGravity += linearVelocity;

        // ////////////////////////////////////////////////////////////////
	}

	void applyTorque(const Eigen::Vector3f& force, const Eigen::Vector3f& rel_pos)
	{
        Eigen::Vector3f torque = rel_pos.cross(force.cwiseProduct(m_linearFactor));
		// m_totalTorque += torque.cwiseProduct(m_angularFactor);
		torque = torque.cwiseProduct(m_angularFactor);
		m_totalTorque += torque;

        // ////////////////////////////////////////////////////////////////
        // // 중력에 의한 COM에서의 각속도 적용
		// m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * dt_;
		
        // m_angularVelocity += m_invInertiaTensorWorld * torque * m_elapsed_time_scale; // m_elapsed_time_scale [ms] :  연산시간에 따른 지연 보상
        Eigen::Vector3f angularVelocity = m_invInertiaTensorWorld * torque * m_elapsed_time_scale;
        m_angularVelocity += angularVelocity;
		m_angularVelocityGravity += angularVelocity;

        // //// TODO: 추후 각속도에 의한 선속도 적용 여부 결정
        m_linearVelocity += angularVelocity.cross(rel_pos);
		m_linearVelocityGravity += angularVelocity.cross(rel_pos);
        // printf("angularVelocity.cross(rel_pos): %f, %f, %f\n", angularVelocity.cross(rel_pos));
        // ////////////////////////////////////////////////////////////////
	}

	void applyForce(const Eigen::Vector3f& force, const Eigen::Vector3f& rel_pos)
	{
		applyCentralForce(force);
		// applyTorque(rel_pos.cross(force.cwiseProduct(m_linearFactor)));
		applyTorque(force, rel_pos);
	}

	const Eigen::Vector3f& getTotalForce() const
	{
		return m_totalForce;
	};

	const Eigen::Vector3f& getTotalTorque() const
	{
		return m_totalTorque;
	};

	const Eigen::Vector3f& getInvInertiaDiagLocal() const
	{
		return m_invInertiaLocal;
	};

	void applyCentralImpulse(const Eigen::Vector3f& impulse)
	{
        Eigen::Vector3f tmp = (impulse.cwiseProduct(m_impulseLinearFactor)) * m_inverseMass;
        // printf("impulse_linearVelocity: %f, %f, %f\n", tmp[0], tmp[1], tmp[2]);
		m_linearVelocity += tmp;
		m_linearVelocityImpulse += tmp;
	}

	// void applyTorqueImpulse(const Eigen::Vector3f& torque)
	// {
    //     Eigen::Vector3f tmp = m_invInertiaTensorWorld * (torque.cwiseProduct(m_impulseAngularFactor));
    //     // printf("impulse_angularVelocity: %f, %f, %f\n", tmp[0], tmp[1], tmp[2]);
	// 	m_angularVelocity += tmp;
	// 	m_angularVelocityImpulse += tmp;
	// }

	void applyTorqueImpulse(const Eigen::Vector3f& impulse, const Eigen::Vector3f& rel_pos)
	{
        Eigen::Vector3f torque = rel_pos.cross(impulse.cwiseProduct(m_impulseLinearFactor));
        Eigen::Vector3f tmp = m_invInertiaTensorWorld * (torque.cwiseProduct(m_impulseAngularFactor));
		m_angularVelocity += tmp;
		m_angularVelocityImpulse += tmp;

        //// TODO: 추후 각속도에 의한 선속도 적용 여부 결정
        m_linearVelocity += tmp.cross(rel_pos);
		m_linearVelocityImpulse += tmp.cross(rel_pos);
	}

    

	void applyImpulse(const Eigen::Vector3f& impulse, const Eigen::Vector3f& rel_pos)
	{
		if (m_inverseMass != float(0.))
		{
			applyCentralImpulse(impulse);
            // applyTorqueImpulse(rel_pos.cross(impulse.cwiseProduct(m_impulseLinearFactor)));
            applyTorqueImpulse(impulse, rel_pos);

		}
	}
    
	void clearForces()
	{
        //// Initialize step force
        m_totalForce = Eigen::Vector3f::Zero(); // reset net force at the end
        m_totalTorque = Eigen::Vector3f::Zero(); // reset net force at the end
	}

	const Eigen::Vector3f& getLinearVelocity() const
	{
		return m_linearVelocity;
	}
	const Eigen::Vector3f& getAngularVelocity() const
	{
		return m_angularVelocity;
	}

	inline void setLinearVelocity(const Eigen::Vector3f& lin_vel)
	{
		m_linearVelocity = lin_vel;
	}

	inline void setAngularVelocity(const Eigen::Vector3f& ang_vel)
	{
		m_angularVelocity = ang_vel;
	}

	Eigen::Vector3f getVelocityInLocalPoint(const Eigen::Vector3f& rel_pos) const
	{
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);
	}
    
	Eigen::Vector3f getVelocityInLocalPointBeforeCollision(const Eigen::Vector3f& rel_pos) const
	{
		return m_prev_linearVelocity + m_prev_angularVelocity.cross(rel_pos);
	}

	float computeAngularImpulseDenominator(const Eigen::Vector3f& axis) const
	{
		// Eigen::Vector3f vec = axis * getInvInertiaTensorWorld();
		Eigen::Vector3f vec = getInvInertiaTensorWorld()*axis;
		return axis.dot(vec);
	}

	void setAngularFactor(const Eigen::Vector3f& angFac)
	{
		// m_updateRevision++;
		m_angularFactor = angFac;
	}

	void setImpulseAngularFactor(const Eigen::Vector3f& angFac)
	{
		// m_updateRevision++;
		m_impulseAngularFactor = angFac;
	}

	void setAngularFactor(float angFac)
	{
		// m_updateRevision++;
		// m_angularFactor.setValue(angFac, angFac, angFac);
        for (int i = 0; i < 3; i++) m_angularFactor[i] = angFac;
        
	}
	const Eigen::Vector3f& getAngularFactor() const
	{
		return m_angularFactor;
	}
};




//////////////////


// struct CollisionPoints {
// 	Eigen::Vector3f A; // Furthest point of A into B
// 	Eigen::Vector3f B; // Furthest point of B into A
// 	Eigen::Vector3f Normal; // B – A normalized
// 	float Depth;    // Length of B – A
// 	bool HasCollision;
// };
 
// struct Transform { // Describes an objects location
// 	Eigen::Vector3f Position;
// 	Eigen::Vector3f Scale;
// 	Eigen::Vector3f Rotation; // rpy
// };

// struct Collider {
// 	virtual CollisionPoints TestCollision(
// 		const Transform* transform,
// 		const Collider* collider,
// 		const Transform* colliderTransform) const = 0;
 
// 	virtual CollisionPoints TestCollision(
// 		const Transform* transform,
// 		const SphereCollider* sphere,
// 		const Transform* sphereTransform) const = 0;
 
// 	virtual CollisionPoints TestCollision(
// 		const Transform* transform,
// 		const PlaneCollider* plane,
// 		const Transform* planeTransform) const = 0;
// };

// struct Object {
// 	float Mass;
// 	Eigen::Vector3f Position;
// 	Eigen::Vector3f Rotation;
// 	Eigen::Vector3f Velocity;
// 	Eigen::Vector3f Force;

//     Eigen::Matrix4f T;

    

// 	// Collider* Collider;
// 	// Transform* Transform;
// };

// struct Collision {
// 	Object* ObjA;
// 	Object* ObjB;
// 	CollisionPoints Points;
// };

// struct CollisionObject {
// protected:
// 	// Transform m_transform;
// 	// Transform m_lastTransform;
// 	// Collider* m_collider;
// 	bool m_isTrigger;
// 	bool m_isStatic;
// 	bool m_isDynamic;
 
// 	std::function<void(Collision&, float)> m_onCollision;
// public:
// 	// Getters & setters for everything, no setter for isDynamic
// };


// class PhysicsWorld {
// private:
// 	std::vector<Object*> m_objects;
// 	Eigen::Vector3f m_gravity = Eigen::Vector3f(0, 0, -9.81f);
 
// public:
// 	void AddObject   (Object* object) { m_objects.push_back(object); }
// 	void RemoveObject(Object* object) { /* ... */ }
  
// 	// void AddSolver   (Solver* solver) { /* ... */ }
// 	// void RemoveSolver(Solver* solver) { /* ... */ }
//     Eigen::VectorXf getPosition(size_t idx) 
//     { 
//         Eigen::VectorXf output(6);
//         output.segment<3>(0) = m_objects[idx]->Position;
//         output.segment<3>(3) = m_objects[idx]->Rotation;
//         return output;
//     }
// 	void Step(
// 		float dt)
// 	{
//         // ResolveCollisions(dt);

// 		for (Object* obj : m_objects) {
// 			obj->Force += obj->Mass * m_gravity; // apply a force
 
// 			obj->Velocity += obj->Force / obj->Mass * dt;
// 			obj->Position += obj->Velocity * dt;
 
// 			obj->Force = Eigen::Vector3f(0, 0, 0); // reset net force at the end
// 		}
// 	}

//     void ResolveCollisions(
// 		float dt)
// 	{
//         // Detect collision
// 		std::vector<Collision> collisions;
// 		for (Object* a : m_objects) {
// 			for (Object* b : m_objects) {
// 				if (a == b) break;
 
//                 //// Check collision
// 				// if (    !a->Collider
// 				// 	|| !b->Collider)
// 				// {
// 				// 	continue;
// 				// }
 
// 				// CollisionPoints points = a->Collider->TestCollision(
// 				// 	a->Transform,
// 				// 	b->Collider,
// 				// 	b->Transform);
 
// 				// if (points.HasCollision) {
// 				// 	collisions.emplace_back(a, b, points);
// 				// }
// 			}
// 		}

// 		// Solve collisions
// 		// for (Solver* solver : m_solvers) {
// 		// 	solver->Solve(collisions, dt);
// 		// }
// 	}
// };


class PhysicsObject {
private:
	// std::vector<Object*> m_objects;
	Eigen::Vector3f m_gravity = Eigen::Vector3f(0, 0, -9.81f);
 
public:

	float Mass;
	Eigen::Vector3f Position;
    Eigen::Vector3f Rotation;
	Eigen::Vector3f dPosition;
	Eigen::Vector3f dRotation;

	Eigen::Vector3f Velocity;
	Eigen::Vector3f Force;

    Eigen::Matrix4f T;


	// void AddObject   (Object* object) { m_objects.push_back(object); }
	// void RemoveObject(Object* object) { /* ... */ }

	void Step(
		float dt)
	{
        Force += Mass * m_gravity; // apply a force

        Velocity += Force / Mass * dt;
        Position += Velocity * dt;
        dPosition = Velocity * dt;

        Force = Eigen::Vector3f(0, 0, 0); // reset net force at the end
	}
	
};


// struct RigidBody
// 	: CollisionObject
// {
// private:
// 	Eigen::Vector3f m_gravity;  // Gravitational acceleration
// 	Eigen::Vector3f m_force;    // Net force
// 	Eigen::Vector3f m_velocity;
 
// 	float m_mass;
// 	bool m_takesGravity; // If the RigidBody will take gravity from the world.
 
// 	float m_staticFriction;  // Static friction coefficient
// 	float m_dynamicFriction; // Dynamic friction coefficient
// 	float m_restitution;     // Elasticity of collisions (bounciness)
 
// public:
// 	// getters & setters
// };

// class Solver {
// public:
// 	virtual void Solve(
// 		std::vector<Collision>& collisions,
// 		float dt) = 0;
// };
// class CollisionWorld {
// protected:
// 	std::vector<CollisionObject*> m_objects;
// 	std::vector<Solver*> m_solvers;
 
// 	std::function<void(Collision&, float)> m_onCollision;
 
// public:
// 	void AddCollisionObject   (CollisionObject* object) { /* ... */ }
// 	void RemoveCollisionObject(CollisionObject* object) { /* ... */ }
 
// 	void AddSolver   (Solver* solver) { /* ... */ }
// 	void RemoveSolver(Solver* solver) { /* ... */ }
 
// 	void SetCollisionCallback(std::function<void(Collision&, float)>& callback) { /* ... */ }
 
// 	void SolveCollisions(
// 		std::vector<Collision>& collisions,
// 		float dt)
// 	{
// 		for (Solver* solver : m_solvers) {
// 			solver->Solve(collisions, dt);
// 		}
// 	}
 
// 	void SendCollisionCallbacks(
// 		std::vector<Collision>& collisions,
// 		float dt)
// 	{
// 		for (Collision& collision : collisions) {
// 			m_onCollision(collision, dt);
 
// 			auto& a = collision.ObjA->OnCollision();
// 			auto& b = collision.ObjB->OnCollision();
 
// 			if (a) a(collision, dt);
// 			if (b) b(collision, dt);
// 		}
// 	}
 
// 	void ResolveCollisions(
// 		float dt)
// 	{
// 		std::vector<Collision> collisions;
// 		std::vector<Collision> triggers;
// 		for (CollisionObject* a : m_objects) {
// 			for (CollisionObject* b : m_objects) {
// 				if (a == b) break;
 
// 				if (    !a->Col()
// 					|| !b->Col())
// 				{
// 					continue;
// 				}
 
// 				CollisionPoints points = a->Col()->TestCollision(
// 					a->Trans(),
// 					b->Col(),
// 					b->Trans());
 
// 				if (points.HasCollision) {
// 					if (    a->IsTrigger()
// 						|| b->IsTrigger())
// 					{
// 						triggers.emplace_back(a, b, points);
// 					}
 
// 					else {
// 						collisions.emplace_back(a, b, points);
// 					}
// 				}
// 			}
// 		}
 
// 		SolveCollisions(collisions, dt); // Don't solve triggers
 
// 		SendCollisionCallbacks(collisions, dt);
// 		SendCollisionCallbacks(triggers, dt);
// 	}
// };

// class DynamicsWorld
// 	: public CollisionWorld
// {
// private:
// 	Eigen::Vector3f m_gravity = Eigen::Vector3f(0, 0, -9.81f);
 
// public:
// 	void AddRigidBody(
// 		RigidBody* RigidBody)
// 	{
// 		if (RigidBody->TakesGravity()) {
// 			RigidBody->SetGravity(m_gravity);
// 		}
 
// 		AddCollisionObject(RigidBody);
// 	}
 
// 	void ApplyGravity() {
// 		for (CollisionObject* object : m_objects) {
// 			if (!object->IsDynamic()) continue;
 
// 			RigidBody* RigidBody = (RigidBody*)object;
// 			RigidBody->ApplyForce(RigidBody->Gravity() * RigidBody->Mass());
// 		}
// 	}
 
// 	void MoveObjects(
// 		float dt)
// 	{
// 		for (CollisionObject* object : m_objects) {
// 			if (!object->IsDynamic()) continue;
 
// 			RigidBody* RigidBody = (RigidBody*)object;
 
// 			Eigen::Vector3f vel = RigidBody->Velocity()
// 					  + RigidBody->Force() / RigidBody->Mass()
// 					  * dt;
 
// 			RigidBody->SetVelocity(vel);

// 			Eigen::Vector3f pos = RigidBody->Position()
// 					  + RigidBody->Velocity()
// 					  * dt;
 
// 			RigidBody->SetPosition(pos);
 
// 			RigidBody->SetForce(Eigen::Vector3f(0, 0, 0));
// 		}
// 	}
 
// 	void Step(
// 		float dt)
// 	{
// 		ApplyGravity();
// 		ResolveCollisions(dt);
// 		MoveObjects(dt);
// 	}
// };
