#include "enum_functions.hpp"

BEGIN_ENUM_VALUE(us_head_positions_t, int16_t)
{       
    DECL_ENUM_VALUE(US_ZERO, 96)    
    DECL_ENUM_VALUE(US_RIGHT, 0) 
    DECL_ENUM_VALUE(US_LEFT, 255)
}
END_ENUM