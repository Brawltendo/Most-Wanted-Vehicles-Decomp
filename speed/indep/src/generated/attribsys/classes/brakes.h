
namespace Attrib
{
namespace Gen
{

struct brakes
{
    char pad_0000[8];
    struct LayoutStruct
    {
        float BRAKE_LOCK[2];
        float BRAKES[2];
        float EBRAKE;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
