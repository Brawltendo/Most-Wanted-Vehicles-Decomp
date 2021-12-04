
struct TableBase
{
	int NumEntries;
	float MinArg;
	float MaxArg;
	float IndexMultiplier;
};

struct Table : TableBase
{
	Table(int numEntries, float min, float max, float indexMul, float* table)
	{
		NumEntries = numEntries;
		MinArg = min;
		MaxArg = max;
		IndexMultiplier = indexMul;
		pTable = table;
	}
	float GetValue(float input);
    float *pTable;
};