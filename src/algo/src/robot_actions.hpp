#include "enum_functions.hpp"

BEGIN_ENUM(robot_actions_t)
{       
    DECL_ENUM_ELEMENT(ADVANCING)    
    DECL_ENUM_ELEMENT(TURNING) 
    DECL_ENUM_ELEMENT(US_SENSING) 
#ifdef DEBUG
    DECL_ENUM_ELEMENT(ARBITRARY) 
#endif
}
END_ENUM