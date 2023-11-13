#include <stdio.h>

double count_to_angle(int motor, int count)
{
    switch (motor)
    {
    case 1:
        return (1 / 125.5) * (count - 32768.0);
        break;
    case 2:
        return (1 / 131.0) * (count - 32768.0);
        break;
    case 3:
        return (1 / 127.7) * (count - 32768.0);
        break;
    case 4:
        return (0.012391573729863692) * (count - 32768.0);
        break;
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}
int ref_to_count(int motor, int ref)
{
    switch (motor)
    {
    case 1:
        return (40.35269645959781 * ref) + 32768;
        break;
    case 2:
        return (14.770677455806219 * ref) + 32768;
        break;
    case 3:
        return (41.6752813118148 * ref) + 32768;
        break;
    case 4:
        return (4.582209206643176 * ref) + 32768;
        break;
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}

int count_to_ref(int motor, int count)
{
    switch (motor)
    {
    case 1:
        return (0.02478) * (count - 32768);
        break;
    case 2:
        return (0.0677) * (count - 32768);
        break;
    case 3:
        return (0.02399) * (count - 32768);
        break;
    case 4:
        return (0.2182353) * (count - 32768);
        break;
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}

int angle_to_ref(int motor, double angle)
{
    switch (motor)
    {
    case 1:
        return int(-3 * angle);
    case 2:
        return int(-9.4 * angle);
    case 3:
        return int(-3.1 * angle);
    case 4:
        return int(-17.61158871 * angle);
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}

double ref_to_angle(int motor, int ref)
{
    switch (motor)
    {
    case 1:
        return ((-1 / 3.0) * ref);
    case 2:
        return ((-1 / 9.4) * ref);
    case 3:
        return ((-1 / 3.1) * ref);
    case 4:
        return (-0.056780795 * ref);
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}