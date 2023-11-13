#include <stdio.h>

/**
 * Function to convert angles into reference values. This function is
 * specific for each arm.
 *
 * TODO: Adjust this implementation according to your arm
 *
 * DOUBT: angles are in DEGREES or in RADIANS?
 **/
double angle_to_ref(int motor, double angle)
{
    switch (motor)
    {
    case 1:
        return (3049 * angle);
    case 2:
        return (3095 * angle);
    case 3:
        return (3686 * angle);
    case 4:
        return (2344 * angle);
    case 5:
        return (2321 * angle);
    case 6:
        return (1138 * angle);
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}

/**
 * Function to convert reference values into angles. This function is
 * specific for each arm.
 *
 * TODO: Adjust this implementation according to your arm
 *
 * DOUBT: angles are in DEGREES or in RADIANS?
 **/
double ref_to_angle(int motor, double ref)
{
    switch (motor)
    {
    case 1:
        return ((1.0 / 3049) * ref);
    case 2:
        return ((1.0 / 3095) * ref);
    case 3:
        return ((1.0 / 3686) * ref);
    case 4:
        return ((1.0 / 2344) * ref);
    case 5:
        return ((1.0 / 2321) * ref);
    case 6:
        return ((1.0 / 1138) * ref);
    default:
        puts("Maximum actionable joint is J4 for them moment");
        break;
    }
    return 0;
}