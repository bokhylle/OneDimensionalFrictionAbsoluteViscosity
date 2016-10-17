#include <cmath>
#include "friction.h"
#include "node.h"
#include <iostream>
using namespace std;

Friction::Friction(double alpha)
{
    m_alpha = alpha;
}

double Friction::calculateFrictionForce(){

    if(m_stuck==false && sign(m_parent->v()) != sign(m_parent->vPrevious()) && sign(m_parent->vPrevious())!=0){//Check if node gets stuck
        m_stuck = true;
    }

    if(m_stuck){//if node is stuck
        if(m_parent->drivingForce() + m_parent->springForce() + m_parent->dampingForce() >= 1 - m_parent->tauBar()){//Unstick if static treshold is reached
            m_stuck = false;
        }else{
            m_parent->setV(0);
            m_frictionForce = -(m_parent->springForce() + m_parent->dampingForce() + m_parent->drivingForce());
        }
    }
    if(!m_stuck){//If sliding, add dynamic friction force
        m_frictionForce = m_parent->tauBar() - m_alpha*m_parent->v();
    }

    return m_frictionForce;
}

Node *Friction::parent() const
{
    return m_parent;
}

void Friction::setParent(std::shared_ptr<Node> node)
{
    m_parent = node.get();
}

bool Friction::stuck() const
{
    return m_stuck;
}

void Friction::setStuck(bool stuck)
{
    m_stuck = stuck;
}

int Friction::sign(double a){
    if(a==0){
        return 0;
    }else{
        return a/std::abs(a);
    }
}
