#include <iostream>
#include "node.h"
#include "friction.h"
using namespace std;

Node::Node(double eta,double tauBar){
    m_eta = eta;
    m_tauBar = tauBar;
}

void Node::calculateForces(){

    //Calculate spring force and damping force
    if(!m_connectionLeft && m_connectionRight){//First node
        m_springForce = m_connectionRight->x() - m_x;
        m_dampingForce = m_eta*(m_connectionRight->v() - m_v);
    }else if (!m_connectionRight && m_connectionLeft){//Last node
        m_springForce = m_connectionLeft->x() - m_x;
        m_dampingForce = m_eta*(m_connectionLeft->v() - m_v);
    }else if(!m_connectionRight && !m_connectionRight){
        m_springForce = 0;
        m_dampingForce = 0;
    }else{
        m_springForce = m_connectionRight->x() - 2*m_x +  m_connectionLeft->x();
        m_dampingForce = m_eta*(m_connectionRight->v() - 2*m_v +  m_connectionLeft->v());
    }

    //Calculate friction forces
    m_frictionForce = m_frictionLaw->calculateFrictionForce();

    //Sum up
    m_tangentialForce = m_springForce + m_dampingForce + m_frictionForce + m_drivingForce;

}

double Node::v() const
{
    return m_v;
}

void Node::setV(double v)
{
    m_v = v;
}

double Node::vPrevious() const
{
    return m_vPrevious;
}

void Node::setVPrevious(double vPrevious)
{
    m_vPrevious = vPrevious;
}

double Node::x() const
{
    return m_x;
}

void Node::setX(double x)
{
    m_x = x;
}

double Node::p() const
{
    return m_p;
}

void Node::setP(double p)
{
    m_p = p;
}

double Node::eta() const
{
    return m_eta;
}

void Node::setEta(double eta)
{
    m_eta = eta;
}

double Node::tauBar() const
{
    return m_tauBar;
}

void Node::setTauBar(double tauBar)
{
    m_tauBar = tauBar;
}

double Node::drivingForce() const
{
    return m_drivingForce;
}

void Node::setDrivingForce(double drivingForce)
{
    m_drivingForce = drivingForce;
}

double Node::springForce() const
{
    return m_springForce;
}

void Node::setSpringForce(double springForce)
{
    m_springForce = springForce;
}

double Node::dampingForce() const
{
    return m_dampingForce;
}

void Node::setDampingForce(double dampingForce)
{
    m_dampingForce = dampingForce;
}

double Node::frictionForce() const
{
    return m_frictionForce;
}

void Node::setFrictionForce(double frictionForce)
{
    m_frictionForce = frictionForce;
}

std::shared_ptr<Friction> Node::frictionLaw() const
{
    return m_frictionLaw;
}

void Node::setFrictionLaw(const std::shared_ptr<Friction> &frictionLaw)
{
    m_frictionLaw = frictionLaw;
    m_frictionLaw->setParent(shared_from_this());
}

std::shared_ptr<Node> Node::connectionLeft() const
{
    return m_connectionLeft;
}

void Node::setConnectionLeft(const std::shared_ptr<Node> &connectionLeft)
{
    m_connectionLeft = connectionLeft;
}

std::shared_ptr<Node> Node::connectionRight() const
{
    return m_connectionRight;
}

void Node::setConnectionRight(const std::shared_ptr<Node> &connectionRight)
{
    m_connectionRight = connectionRight;
}

double Node::tangentialForce() const
{
    return m_tangentialForce;
}

void Node::setTangentialForce(double tangentialForce)
{
    m_tangentialForce = tangentialForce;
}

