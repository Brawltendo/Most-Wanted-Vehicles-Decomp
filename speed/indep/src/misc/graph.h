#include "math/vector.h"

struct GraphBase
{
    //virtual void GetValue();
    //virtual void Blend();

    UMath::Vector2 *pGraph;
    int iGraphLength;
};

template<typename T>
struct tGraph : GraphBase
{
    //void GetValue<T>(T& graphOutput, const float graphInput);
    //void Blend<T>(T& dest, const T* next, const T* current, const float blend);
};

template<>
struct tGraph<float> : GraphBase
{
	tGraph<float>(UMath::Vector2 graph[], int length)
	{
		pGraph = graph;
		iGraphLength = length;
	}
    void GetValue(float& graphOutput, const float graphInput);
    void Blend(float& dest, const float* next, const float* current, const float blend);
};