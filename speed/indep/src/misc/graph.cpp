#include "misc/graph.h"

// <@>PRINT_ASM
float Graph::GetValue(const float x)
{
    const int length = iGraphLength;

	if (length > 1)
	{
		if (x <= pGraph[0].x)
			return pGraph[0].y;
		if (x >= pGraph[length - 1].x)
			return pGraph[length - 1].y;
		int graphInd = 0;
		if (length - 1 > 0)
		{
			for (UMath::Vector2* graphIt = pGraph; x >= graphIt[0].x && x < graphIt[1].x; ++graphIt)
			{
				if (++graphInd < length - 1)
					continue;
				//else
					return pGraph[0].y;
			}
			
			//float curY  = pGraph[graphInd].y;
			float diffY = pGraph[graphInd + 1].y - pGraph[graphInd].y;
			float diffX = pGraph[graphInd + 1].x - pGraph[graphInd].x;
			if (fabsf(diffX) > FLT_EPSILON)
				return diffY * ((x - pGraph[graphInd].x) / diffX) + pGraph[graphInd].y;
			else
				return diffY * 0.5f + pGraph[graphInd].y;
		}
		return pGraph[0].y;

		
	}
	return pGraph[0].y;
}

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