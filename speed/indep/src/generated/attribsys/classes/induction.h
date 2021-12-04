
namespace Attrib
{
namespace Gen
{

struct induction
{
    char pad_0000[8];
    struct LayoutStruct
    {
        float LOW_BOOST;
		float SPOOL_TIME_DOWN;
		float VACUUM;
		float SPOOL;
		float SPOOL_TIME_UP;
		float PSI;
		float HIGH_BOOST;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib