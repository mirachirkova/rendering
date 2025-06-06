#include "common.h"


std::ostream& operator<<(std::ostream& stream, LiteMath::float3 vector)
{
    stream << "{" << vector.x << ", " << vector.y << ", " << vector.z << "}";
    return stream;
}