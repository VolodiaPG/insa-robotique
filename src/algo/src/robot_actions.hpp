#include "enum_to_string.hpp"

BEGIN_ENUM(robot_actions_t)
{       
    DECL_ENUM_ELEMENT(ADVANCING)    
    DECL_ENUM_ELEMENT(TURNING) 
    DECL_ENUM_ELEMENT(US_SENSING) 
}
END_ENUM