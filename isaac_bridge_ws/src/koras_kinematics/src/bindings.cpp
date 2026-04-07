#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "robot_model.hpp"

namespace py = pybind11;

// Helper: ModelMatrix → Python list
static std::vector<double> mm2vec(const ModelMatrix& m) {
    return std::vector<double>(m.element_.begin(), m.element_.end());
}

// Helper: 6-element list → ModelMatrix (6x1)
static ModelMatrix vec2mm6(const std::vector<double>& v) {
    ModelMatrix m(6, 1);
    for (int i = 0; i < 6; i++) m.element_[i] = v[i];
    return m;
}

PYBIND11_MODULE(koras_kinematics, mod) {
    mod.doc() = "Koras robot FK/IK/Dynamics (pybind11 wrapper)";

    py::class_<RobotModel>(mod, "RobotModel")
        .def(py::init<>())

        // DH 파라미터 설정: list[6][4] (alpha, a, d, theta)
        .def("setDH", [](RobotModel& self, py::list dh_list) {
            double dh[JS_DOF][4];
            for (int i = 0; i < JS_DOF; i++)
                for (int j = 0; j < 4; j++)
                    dh[i][j] = py::cast<double>(dh_list[i].cast<py::list>()[j]);
            self.setDH(dh);
        })

        // 동역학 파라미터 설정
        .def("setDynamicParameters", [](RobotModel& self,
            std::vector<double> mass,
            py::list com_list,
            py::list inertia_list,
            std::vector<double> inertia_rotor)
        {
            double m[JS_DOF], com[JS_DOF][3], I[JS_DOF][9], ir[JS_DOF];
            for (int i = 0; i < JS_DOF; i++) {
                m[i] = mass[i];
                ir[i] = inertia_rotor[i];
                for (int j = 0; j < 3; j++)
                    com[i][j] = py::cast<double>(com_list[i].cast<py::list>()[j]);
                for (int j = 0; j < 9; j++)
                    I[i][j] = py::cast<double>(inertia_list[i].cast<py::list>()[j]);
            }
            self.setDynamicParameters(m, com, I, ir);
        })

        // TCP 설정
        .def("setTcp", [](RobotModel& self, std::vector<double> tcp) {
            double t[CS_DOF];
            for (int i = 0; i < CS_DOF; i++) t[i] = tcp[i];
            self.setTcp(t);
        })

        // FK: q_deg[6] → [x,y,z,rx,ry,rz]
        .def("fwdKine", [](RobotModel& self, std::vector<double> q_deg) {
            double q[JS_DOF], x[6];
            for (int i = 0; i < JS_DOF; i++) q[i] = q_deg[i];
            self.setq(q);
            self.fwdKine(x);
            return std::vector<double>(x, x + 6);
        })

        // FK ZYZ: q_deg[6] → [x,y,z,rz1,ry,rz2]
        .def("fwdKineZYZ", [](RobotModel& self, std::vector<double> q_deg) {
            double q[JS_DOF], x[6];
            for (int i = 0; i < JS_DOF; i++) q[i] = q_deg[i];
            self.setq(q);
            self.fwdKineZYZ(x);
            return std::vector<double>(x, x + 6);
        })

        // IK: x_target[6](mm,deg), q_current[6](deg) → (q_target[6], success)
        .def("inverseKinematics", [](RobotModel& self,
            std::vector<double> x_target,
            std::vector<double> q_current)
        {
            std::vector<double> q_out(JS_DOF, 0.0);
            bool ok = self.inverseKinematics(x_target, q_current, q_out);
            return py::make_tuple(q_out, ok);
        })

        // 중력 토크: q_deg[6] → torque[6] (Nm)
        .def("calGravityTorque", [](RobotModel& self, std::vector<double> q_deg) {
            double q[JS_DOF], tau[JS_DOF];
            for (int i = 0; i < JS_DOF; i++) q[i] = q_deg[i];
            self.calGravityTorque(q, tau);
            return std::vector<double>(tau, tau + JS_DOF);
        })

        // 관성 토크: q_deg[6], qdd_deg[6] → torque[6] (Nm)
        .def("calInertiaTorque", [](RobotModel& self,
            std::vector<double> q_deg,
            std::vector<double> qdd_deg)
        {
            double q[JS_DOF], qdd[JS_DOF], tau[JS_DOF];
            for (int i = 0; i < JS_DOF; i++) { q[i] = q_deg[i]; qdd[i] = qdd_deg[i]; }
            self.calInertiaTorque(q, qdd, tau);
            return std::vector<double>(tau, tau + JS_DOF);
        })

        // 코리올리 토크: q_deg[6], qd_deg[6] → torque[6] (Nm)
        .def("calCoriolisTorque", [](RobotModel& self,
            std::vector<double> q_deg,
            std::vector<double> qd_deg)
        {
            double q[JS_DOF], qd[JS_DOF], tau[JS_DOF];
            for (int i = 0; i < JS_DOF; i++) { q[i] = q_deg[i]; qd[i] = qd_deg[i]; }
            self.calCoriolisTorque(q, qd, tau);
            return std::vector<double>(tau, tau + JS_DOF);
        })

        // 마찰 토크 설정 및 계산
        .def("setFrictionModel", [](RobotModel& self,
            std::vector<double> coulomb,
            py::list viscous_list,
            std::vector<double> deadzone,
            std::vector<int> n_elem)
        {
            double c[JS_DOF], v[JS_DOF][5], dz[JS_DOF];
            int ne[JS_DOF];
            for (int i = 0; i < JS_DOF; i++) {
                c[i] = coulomb[i];
                dz[i] = deadzone[i];
                ne[i] = n_elem[i];
                for (int j = 0; j < 5; j++)
                    v[i][j] = py::cast<double>(viscous_list[i].cast<py::list>()[j]);
            }
            self.setFrictionModel(c, v, dz, ne);
        })

        .def("calFrictionTorque", [](RobotModel& self,
            std::vector<double> q_deg,
            std::vector<double> qd_deg)
        {
            double q[JS_DOF], qd[JS_DOF], tau[JS_DOF];
            for (int i = 0; i < JS_DOF; i++) { q[i] = q_deg[i]; qd[i] = qd_deg[i]; }
            self.calFrictionTorque(q, qd);
            self.getFrictionTorque(tau);
            return std::vector<double>(tau, tau + JS_DOF);
        });
}
