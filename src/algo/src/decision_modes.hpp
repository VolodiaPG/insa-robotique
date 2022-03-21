#include "enum_to_string.hpp"

BEGIN_ENUM(robot_decision_mode_t)
{       
    DECL_ENUM_ELEMENT(LINE_FOLLOWING)    
    DECL_ENUM_ELEMENT(DEPTH_SENSING) 
    DECL_ENUM_ELEMENT(ROTATION)
}
END_ENUM