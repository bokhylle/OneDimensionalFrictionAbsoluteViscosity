#include <vector>
#include <memory>
#ifndef NODE_H
#define NODE_H

class Node
        : public std::enable_shared_from_this<Node>
{
public:
    Node(double eta,double tau);

    void calculateForces();
    void updateTimeDependentExternalForces();

    double v() const;
    void setV(double v);

    double vPrevious() const;
    void setVPrevious(double vPrevious);

    double x() const;
    void setX(double x);

    double p() const;
    void setP(double p);

    double eta() const;
    void setEta(double eta);

    double tauBar() const;
    void setTauBar(double tauBar);

    double drivingForce() const;
    void setDrivingForce(double drivingForce);

    double springForce() const;
    void setSpringForce(double springForce);

    double dampingForce() const;
    void setDampingForce(double dampingForce);

    double frictionForce() const;
    void setFrictionForce(double frictionForce);

    std::shared_ptr<class Friction> frictionLaw() const;
    void setFrictionLaw(const std::shared_ptr<class Friction> &frictionLaw);

    std::shared_ptr<class Node> connectionLeft() const;
    void setConnectionLeft(const std::shared_ptr<class Node> &connectionLeft);

    std::shared_ptr<class Node> connectionRight() const;
    void setConnectionRight(const std::shared_ptr<class Node> &connectionRight);

    double tangentialForce() const;
    void setTangentialForce(double tangentialForce);

private:
    double m_v = 0;
    double m_vPrevious = 0;
    double m_x = 0;
    double m_p;
    double m_eta;
    double m_tauBar;

    double m_springForce = 0;
    double m_dampingForce = 0;
    double m_frictionForce = 0;
    double m_drivingForce = 0;

    double m_tangentialForce = 0;

    std::shared_ptr<class Friction> m_frictionLaw = nullptr;
    std::shared_ptr<class Node> m_connectionLeft = nullptr;
    std::shared_ptr<class Node> m_connectionRight = nullptr;

};

#endif // NODE_H
