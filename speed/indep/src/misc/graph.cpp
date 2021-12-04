#include "misc/graph.h"

// MATCHING
void tGraph<float>::GetValue(float& out, const float in)
{
    const int length = iGraphLength;

    if (length > 1)
    {
        if (in <= pGraph[0].x)
        	out = pGraph[0].y;
        else if (in >= pGraph[length - 1].x)
			out = pGraph[length - 1].y;
        else
		{
			int i = 0;
			const int maxInd = length - 1;
            if (maxInd > 0)
            {
				UMath::Vector2* p = pGraph;
				while (!(in >= p->x) || !(in < p[1].x))
				{
					++i;
					++p;
					if (i >= maxInd)
						return;
				}
                const float blend = (in - pGraph[i].x) / (pGraph[i+1].x - pGraph[i].x);
				Blend(out, &pGraph[i+1].y, &pGraph[i].y, blend);
            }
		}
    }
    else if (length > 0)
        out = pGraph[0].y;
}

// MATCHING
void tGraph<float>::Blend(float& dest, const float* next, const float* current, const float blend)
{
    dest = (1.f - blend) * *current + blend * *next;
}