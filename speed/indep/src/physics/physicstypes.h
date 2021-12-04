

struct AxlePair
{
    union {
        struct {
            float Front;
            float Rear;
        };
        float Pair[2];
    };
};