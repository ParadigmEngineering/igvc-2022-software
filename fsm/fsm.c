#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

typedef enum
{
    BOOT,
    AUTONOMOUS,
    MANUAL
} State;

void get_next_state(char input, State **state);

int main()
{
    char input;
    State * state;
    State tmp;

    State boot = BOOT;

    state = &boot;

    while (1)
    {
        printf("\nCurrent state is: %d\n", *state);
        printf("Please enter character input: ");
        scanf("%c", &input);
        if (input == '\n' || input == EOF)
        {
            continue;
        }
        if (input == 'q' || input == 'Q')
        {
            break;
        }
        get_next_state(input, &state);
        tmp = *state;
        fflush(stdin);
        state = &tmp;
    }
}

void get_next_state(char input, State **state)
{
    State boot = BOOT;
    State autonomous = AUTONOMOUS;
    State manual = MANUAL;

    int curr_state = (int)**state;

    if (curr_state == (int)boot)
    {
        if (input == 'P' || input == 'p' || input == 'W' || input == 'w')
        {
            *state = &boot;
        }

        else if (input == 'A' || input == 'a')
        {
            *state = &autonomous;
        }

        else if (input == 'M' || input == 'm')
        {
            *state = &manual;
        }
    }

    else if (curr_state == (int)autonomous)
    {
        if (input == 'P' || input == 'p' || input == 'W' || input == 'w')
        {
            *state = &boot;
        }

        else
        {
            *state = &autonomous;
        }
    }

    else if (curr_state == (int)manual)
    {
        if (input == 'P' || input == 'p' || input == 'W' || input == 'w')
        {
            *state = &boot;
        }

        else
        {
            *state = &manual;
        }
    }

    else
    {
        printf("Error: Unknown State\n");
    }
}
