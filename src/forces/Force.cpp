#include "Force.h"

std::vector<Force*> Force::_forces;
std::vector<Force*> Force::_mouse_forces;

void Force::AddForce(Force* force){
    Force::_forces.emplace_back(force);
}

void Force::AddMouseForce(Force* mouse_force){
    Force::_mouse_forces.emplace_back(mouse_force);
}